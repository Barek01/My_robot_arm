from tinymovr.units import get_registry

ureg = get_registry()
mA = ureg.milliampere
rad = ureg.radian
s = ureg.second

print(ureg('rad/s'))