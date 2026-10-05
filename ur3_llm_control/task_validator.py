"""Plan Validator: kiem tra plan cua LLM truoc khi cho robot chay.

1. Dung dinh dang {"plan": [...]}
2. Skill nam trong danh sach cho phep, dung tham so (khong co joint, trajectory...)
3. Object / zone ton tai; object phai duoc camera nhin thay
4. Logic: khong pick khi dang cam vat, chi place vat dang cam, chi dung camera khi tay trong,
   ket thuc tay phai trong
5. An toan: truoc place(x, zone) phai co clear_zone(zone) -> neu thieu thi tu chen vao
"""

# Danh sach skill LLM duoc dung: ten -> tham so
SKILLS = {
    'detect_objects': [],
    'check_zone': ['zone'],
    'find_object': ['object'],
    'clear_zone': ['zone'],
    'pick': ['object'],
    'place': ['object', 'zone'],
    'place_free': ['object'],
    'home': [],
}

MAX_STEPS = 30


def to_str(s):
    """{'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'} -> place(red_cube, zone_b)"""
    args = [str(s.get(k)) for k in SKILLS.get(s.get('skill'), [])]
    return f"{s.get('skill')}({', '.join(args)})"


def clean(v):
    return str(v).strip().lower().replace(' ', '_').replace('-', '_')


def check(raw, sc):
    """Tra ve (plan, errs). errs rong = hop le."""
    if not isinstance(raw, dict) or not isinstance(raw.get('plan'), list):
        return [], ['output phai co dang {"plan": [...]}']
    steps = raw['plan']
    if not steps:
        return [], ['LLM tu choi: ' + str(raw.get('error', 'plan rong'))]
    if len(steps) > MAX_STEPS:
        return [], [f'plan qua dai ({len(steps)} buoc)']

    plan, errs = [], []
    for i, st in enumerate(steps, 1):
        if not isinstance(st, dict):
            errs.append(f'buoc {i}: khong phai object JSON')
            continue
        sk = clean(st.get('skill', ''))
        if sk not in SKILLS:
            errs.append(f'buoc {i}: skill "{sk}" khong duoc phep')
            continue
        keys = SKILLS[sk]
        extra = [k for k in st if k != 'skill' and k not in keys]
        if extra:
            # vd LLM sinh "joints", "trajectory" -> tu choi
            errs.append(f'buoc {i}: tham so khong hop le {extra} cho {sk}')
            continue
        s = {'skill': sk}
        for k in keys:
            s[k] = clean(st.get(k, ''))
        if 'object' in s and s['object'] not in sc.objs:
            errs.append(f'buoc {i}: INVALID_OBJECT "{s["object"]}"')
            continue
        if 'object' in s and sc.pos[s['object']] is None and sc.held != s['object']:
            errs.append(f'buoc {i}: OBJECT_NOT_FOUND "{s["object"]}" (camera khong thay)')
            continue
        if 'zone' in s and s['zone'] not in sc.zones:
            errs.append(f'buoc {i}: INVALID_ZONE "{s["zone"]}"')
            continue
        plan.append(s)
    if errs:
        return plan, errs

    # chay thu logic tay gap
    held = sc.held
    for i, s in enumerate(plan, 1):
        sk = s['skill']
        if sk == 'pick':
            if held:
                errs.append(f'buoc {i}: pick({s["object"]}) khi dang cam {held}')
            held = s['object']
        elif sk in ('place', 'place_free'):
            if held != s['object']:
                errs.append(f'buoc {i}: {to_str(s)} nhung dang cam {held}')
            held = None
        elif sk in ('detect_objects', 'check_zone', 'clear_zone') and held:
            errs.append(f'buoc {i}: {to_str(s)} khi dang cam {held} (camera can tay trong)')
    if held:
        errs.append(f'plan ket thuc khi van con cam {held}')
    if errs:
        return plan, errs
    return add_safety(plan), []


def add_safety(plan):
    """Them cac buoc an toan neu LLM quen (danh dau note='auto'):
    - detect_objects() o dau plan
    - clear_zone(zone) truoc pick(x) neu sau do place(x, zone) ma chua don zone
    - home() o cuoi plan"""
    out = []
    if plan[0]['skill'] != 'detect_objects':
        out.append({'skill': 'detect_objects', 'note': 'auto'})
    cleared = set()
    for i, s in enumerate(plan):
        if s['skill'] == 'clear_zone':
            cleared.add(s['zone'])
        if s['skill'] == 'pick':
            for t in plan[i + 1:]:
                if t['skill'] in ('place', 'place_free') and t['object'] == s['object']:
                    z = t.get('zone')
                    if z and z not in cleared:
                        out.append({'skill': 'clear_zone', 'zone': z, 'note': 'auto'})
                        cleared.add(z)
                    break
        if s['skill'] in ('place', 'place_free', 'pick'):
            cleared.discard(s.get('zone'))
        out.append(dict(s))
    if out[-1]['skill'] != 'home':
        out.append({'skill': 'home', 'note': 'auto'})
    return out
