"""Chay plan da duoc validator duyet va in ket qua."""
import json

from .task_validator import to_str

BAR = '=' * 56
OK_ST = ('SUCCESS', 'SKIPPED')


def next_place(plan, i, obj):
    # tim buoc place obj ngay sau pick o vi tri i
    for j in range(i + 1, len(plan)):
        if plan[j]['skill'] in ('place', 'place_free') and plan[j]['object'] == obj:
            return j
        if plan[j]['skill'] == 'pick':
            return None
    return None


def do_step(s, sk, keep=None):
    n = s['skill']
    if n == 'home':
        return sk.home()
    if n == 'detect_objects':
        return sk.detect_objects()
    if n == 'check_zone':
        return sk.check_zone(s['zone'])
    if n == 'find_object':
        return sk.find_object(s['object'])
    if n == 'clear_zone':
        return sk.clear_zone(s['zone'], keep)
    if n == 'pick':
        return sk.pick(s['object'])
    if n == 'place':
        return sk.place(s['object'], s['zone'])
    if n == 'place_free':
        return sk.place_free(s['object'])
    return 'FAILED'


def line(lb, st):
    # vd: pick(red_cube) ........ SUCCESS
    return lb + ' ' + '.' * max(2, 34 - len(lb)) + ' ' + st


def run(plan, sk, log=print):
    """Chay tung buoc, gap loi thi dung. Tra ve (ok, res)."""
    ok = True
    res = []
    skip = []
    for i in range(len(plan)):
        s = plan[i]
        lb = to_str(s)
        if s.get('note') == 'auto':
            lb += ' [auto]'

        if not ok:
            st = 'NOT RUN'
        elif i in skip:
            st = 'SKIPPED'
        else:
            # vat se dat vao zone nay -> clear_zone khong don no
            keep = None
            if s['skill'] == 'clear_zone':
                for t in plan[i + 1:]:
                    if t['skill'] == 'place' and t['zone'] == s['zone']:
                        keep = t['object']
                        break

            # vat da o dung zone -> bo qua pick/place
            if s['skill'] == 'pick':
                j = next_place(plan, i, s['object'])
                if j is not None and plan[j]['skill'] == 'place':
                    z = plan[j]['zone']
                    if sk.sc.where(s['object']) == z:
                        skip.append(j)
                        log(f'    {s["object"]} da o {z} -> bo qua pick/place')
                        log(line(lb, 'SKIPPED'))
                        res.append((lb, 'SKIPPED'))
                        continue

            # chay skill
            try:
                st = do_step(s, sk, keep)
            except Exception as e:
                log(f'    loi: {e}')
                st = 'FAILED'
            if st not in OK_ST:
                ok = False
        log(line(lb, st))
        res.append((lb, st))
    return ok, res


def fix_text(s):
    # bo byte loi UTF-8
    return s.encode('utf-8', 'surrogateescape').decode('utf-8', 'ignore').strip()


def show_state(sc, log):
    st = sc.state()
    for o in st:
        p = sc.pos[o]
        xy = ''
        if p:
            xy = f' ({p[0]:.2f}, {p[1]:.2f})'
        log(f'  {o:12s} {st[o]}{xy}')
    zs = sc.zone_state()
    txt = []
    for z in zs:
        txt.append(z + '=' + zs[z])
    log('  zones: ' + ', '.join(txt))


def run_cmd(cmd, pl, sk, log=print):
    """Lenh -> camera -> LLM -> validator -> chay."""
    cmd = fix_text(cmd)
    rep = {'command': cmd, 'plan': [], 'status': 'TASK FAILED'}
    log(BAR)
    log('USER COMMAND:')
    log(cmd)
    log('')

    # 1. camera
    if sk.detect_objects() != 'SUCCESS':
        log('CAMERA ERROR: khong lay duoc trang thai moi truong')
        log(BAR)
        return rep
    log('CAMERA:')
    show_state(sk.sc, log)
    log('')

    # 2. LLM + validator
    try:
        plan, errs, ans = pl.plan(cmd)
    except RuntimeError as e:
        log(f'LLM ERROR: {e}')
        rep['status'] = 'TASK REJECTED'
        log(BAR)
        return rep

    if errs:
        log('LLM PLAN: REJECTED')
        for e in errs:
            log('  - ' + e)
        log('  LLM tra loi: ' + ans.strip()[:300])
        log('')
        log('TASK REJECTED')
        rep['status'] = 'TASK REJECTED'
        log(BAR)
        return rep

    # in plan
    rep['plan'] = plan
    log('LLM PLAN:')
    js = []
    for i in range(len(plan)):
        s = plan[i]
        txt = f'{i + 1}. {to_str(s)}'
        if s.get('note') == 'auto':
            txt += '   [auto: validator them vao]'
        log(txt)
        d = dict(s)
        d.pop('note', None)
        js.append(d)
    log('')
    log('JSON: ' + json.dumps({'plan': js}))
    log('')

    # 3. chay plan
    log('EXECUTION:')
    ok, res = run(plan, sk, log)
    rep['steps'] = res
    if ok:
        rep['status'] = 'TASK SUCCESS'
    else:
        rep['status'] = 'TASK FAILED'
    log('')
    log(rep['status'])

    # 4. camera kiem tra lai
    if ok and sk.detect_objects() == 'SUCCESS':
        log('CAMERA (sau khi lam):')
        show_state(sk.sc, log)
    log(BAR)
    return rep
