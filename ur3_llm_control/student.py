"""Ca nhan hoa: P = (2 so cuoi MSSV) mod 6."""

# P -> mau o Zone A, B, C
TB = {
    0: ('red', 'yellow', 'blue'),
    1: ('red', 'blue', 'yellow'),
    2: ('yellow', 'red', 'blue'),
    3: ('yellow', 'blue', 'red'),
    4: ('blue', 'red', 'yellow'),
    5: ('blue', 'yellow', 'red'),
}


def get_p(sid):
    return int(str(sid)[-2:]) % 6


def assign(sid):
    # {'zone_a': 'red_cube', ...}
    c = TB[get_p(sid)]
    return {'zone_a': c[0] + '_cube', 'zone_b': c[1] + '_cube', 'zone_c': c[2] + '_cube'}


def info(name, sid):
    a = assign(sid)
    s = f'Student: {name} ({sid})\n'
    s += f'P = {str(sid)[-2:]} mod 6 = {get_p(sid)}\n'
    for z in a:
        s += f'  Zone {z[-1].upper()} -> {a[z]}\n'
    return s.rstrip()
