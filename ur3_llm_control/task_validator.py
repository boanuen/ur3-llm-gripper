"""Kiem tra plan cua LLM truoc khi cho robot chay."""

# skill duoc phep: ten -> tham so
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
    # {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'} -> place(red_cube, zone_b)
    n = s.get('skill')
    args = []
    for k in SKILLS.get(n, []):
        args.append(str(s.get(k)))
    return f"{n}({', '.join(args)})"


def clean(v):
    return str(v).strip().lower().replace(' ', '_').replace('-', '_')


def check(raw, sc):
    """Tra ve (plan, errs). errs rong = hop le."""
    # 1. dinh dang
    if not isinstance(raw, dict) or not isinstance(raw.get('plan'), list):
        return [], ['output phai co dang {"plan": [...]}']
    steps = raw['plan']
    if not steps:
        return [], ['LLM tu choi: ' + str(raw.get('error', 'plan rong'))]
    if len(steps) > MAX_STEPS:
        return [], [f'plan qua dai ({len(steps)} buoc)']

    # 2. skill, tham so, object, zone
    plan = []
    errs = []
    for i in range(1, len(steps) + 1):
        st = steps[i - 1]
        if not isinstance(st, dict):
            errs.append(f'buoc {i}: khong phai object JSON')
            continue
        sk = clean(st.get('skill', ''))
        if sk not in SKILLS:
            errs.append(f'buoc {i}: skill "{sk}" khong duoc phep')
            continue
        keys = SKILLS[sk]
        bad = []
        for k in st:
            if k != 'skill' and k not in keys:
                bad.append(k)
        if bad:
            # vd "joints", "trajectory"
            errs.append(f'buoc {i}: tham so khong hop le {bad} cho {sk}')
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

    # 3. chay thu logic tay gap
    held = sc.held
    for i in range(1, len(plan) + 1):
        s = plan[i - 1]
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

    # 4. them buoc an toan
    return add_safety(plan), []


def add_safety(plan):
    """Them detect_objects dau, clear_zone truoc pick, home cuoi (note='auto')."""
    out = []
    if plan[0]['skill'] != 'detect_objects':
        out.append({'skill': 'detect_objects', 'note': 'auto'})
    done = []           # zone da don
    for i in range(len(plan)):
        s = plan[i]
        if s['skill'] == 'clear_zone' and s['zone'] not in done:
            done.append(s['zone'])
        # pick(x) ma sau do place(x, zone) chua don -> chen clear_zone
        if s['skill'] == 'pick':
            for t in plan[i + 1:]:
                if t['skill'] in ('place', 'place_free') and t['object'] == s['object']:
                    z = t.get('zone')
                    if z and z not in done:
                        out.append({'skill': 'clear_zone', 'zone': z, 'note': 'auto'})
                        done.append(z)
                    break
        if s['skill'] in ('place', 'place_free', 'pick'):
            if s.get('zone') in done:
                done.remove(s.get('zone'))
        out.append(dict(s))
    if out[-1]['skill'] != 'home':
        out.append({'skill': 'home', 'note': 'auto'})
    return out
