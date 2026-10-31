"""Offline FT calibration arithmetic; no serial or hardware access."""
import copy,hashlib,json,math
from pathlib import Path
class HandCalibration:
    def __init__(self, config, calibration):
        self.names = list(config['joint_ids'])
        self.ids = [abs(config['joint_to_motor_map'][n]) for n in self.names]
        if len(self.names) != 16 or len(set(self.names)) != 16 or set(self.ids) != set(range(1, 17)):
            raise ValueError('hand must contain exactly motors 1–16; wrist excluded')
        if calibration.get('calibrated') is not True:
            raise ValueError('hand calibration is incomplete')
        self.rom = [config['joint_roms'][n] for n in self.names]
        self.inverted = [config['joint_to_motor_map'][n] < 0 for n in self.names]
        self.limits = [calibration['motor_limits'][i] for i in self.ids]
        self.ratios = [calibration['joint_to_motor_ratios'][i] for i in self.ids]
        self.wrap = None
        for (lo, hi), limits, ratio in zip(self.rom, self.limits, self.ratios):
            if len(limits)!=2 or not all(math.isfinite(v) for v in [lo, hi, *limits, ratio]) or hi <= lo or ratio <= 0 or limits[1]<=limits[0]:
                raise ValueError('invalid hand calibration')
            if not math.isclose(limits[1]-limits[0],ratio*(hi-lo),rel_tol=1e-6,abs_tol=1e-7):
                raise ValueError('hand config/calibration motor ratio differs from joint ROM')

    def initialize_wrap(self, positions, preserve=False):
        if self.wrap is None or not preserve:self.wrap=[None]*16
        for i,(p,l) in enumerate(zip(positions,self.limits)):
            if p is not None and math.isfinite(p) and self.wrap[i] is None:
                self.wrap[i]=-2*math.pi if p<min(l)-.25*math.pi else 2*math.pi if p>max(l)+.25*math.pi else 0.

    def to_joint(self, positions):
        if self.wrap is None:raise ValueError('wrap has not been measured')
        return [None if p is None or w is None else rom[1]-(p-w-l[0])/r if inv else rom[0]+(p-w-l[0])/r
                for p,w,l,r,inv,rom in zip(positions,self.wrap,self.limits,self.ratios,self.inverted,self.rom)]

    def to_motor(self,q,mask=None):
        if self.wrap is None or len(q)!=16:raise ValueError('hand feedback/wrap unavailable')
        result=[]
        for v,rom,inv,r,l,w,use in zip(q,self.rom,self.inverted,self.ratios,self.limits,self.wrap,mask or [True]*16):
            if not use:result.append(None);continue
            if v is None or w is None or not math.isfinite(v) or not rom[0]-1e-7<=v<=rom[1]+1e-7:
                raise ValueError('hand target outside calibrated joint ROM or wrap unavailable')
            result.append(l[0]+(rom[1]-v if inv else v-rom[0])*r+w)
        return result

def load_specs(root):
    """Load one versioned hand bundle for encoder conversion and model mapping."""
    settings = json.loads((Path(root)/'configs/hand_hardware.json').read_text())
    if set(settings['hands'])!={'left','right'} or len({v['serial'] for v in settings['hands'].values()})!=2:
        raise ValueError('双手必须使用两个不同USB序列号')
    import yaml
    mapping = json.loads((Path(root)/'configs/hand_mapping.json').read_text())['joints']
    specs, models = {}, {}
    for side, bus in settings['hands'].items():
        bus=dict(bus,calibration_revision=settings.get('calibration_revision'))
        settings['hands'][side]=bus
        directory = Path(bus['model_path'])
        if not directory.is_absolute():directory=Path(root)/directory
        files = {n: (directory/n).read_bytes() for n in ('config.yaml', 'calibration.yaml')}
        hashes={n:hashlib.sha256(v).hexdigest() for n,v in files.items()}
        model = HandCalibration(*(yaml.safe_load(files[n]) for n in ('config.yaml', 'calibration.yaml')))
        group = side+'_hand'; models[group] = model
        byname = {s['source_name']: s for s in mapping if s['side'] == side}
        specs[group] = []
        for name, mid, rom in zip(model.names, model.ids, model.rom):
            spec = copy.deepcopy(byname[name])
            if settings.get('calibration_revision'):
                expected_direction=-1 if model.inverted[model.names.index(name)] else 1
                if (spec.get('hardware_calibration_revision')!=settings['calibration_revision'] or
                    spec.get('motor_id')!=mid or spec.get('motor_direction')!=expected_direction or
                    spec.get('parameter_sha256')!=hashes or
                    any(abs(math.radians(v)-q)>1e-6 for v,q in zip(rom,spec['real_limits_rad']))):
                    raise ValueError('手部校准文件与映射版本不一致：'+group+'/'+name+'；请统一同步校准配置')
            spec.update(source_joint=name, motor_id=mid, source_unit='deg',
                        real_limits_rad=[math.radians(v) for v in rom],
                        source_topic='/unified8092/'+group+'/feedback',
                        hardware_serial=bus['serial'],
                        parameter_sha256=hashes)
            specs[group].append(spec)
    return settings, specs, models
