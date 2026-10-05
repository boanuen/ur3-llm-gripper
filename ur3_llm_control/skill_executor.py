"""Skill Executor: chay plan da duoc validator duyet va in ket qua ra terminal.

Luong: camera -> LLM -> validator -> chay tung skill -> camera kiem tra lai ket qua.
"""
import json

from .task_validator import to_str

BAR = '=' * 56
OK_ST = ('SUCCESS', 'SKIPPED')


def next_place(plan, i, obj):
    """Buoc place(obj, ...) / place_free(obj) ngay sau pick(obj) o vi tri i."""
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


def run(plan, sk, log=print):
    """Chay tung buoc, dung lai khi co loi. Tra ve (ok, [(buoc, trang thai)])."""
    ok, res, skip = True, [], set()
    for i, s in enumerate(plan):
        lb = to_str(s) + (' [auto]' if s.get('note') == 'auto' else '')
        if not ok:
            st = 'NOT RUN'
        elif i in skip:
            st = 'SKIPPED'
        else:
            keep = None
            if s['skill'] == 'clear_zone':      # vat sap dat vao zone nay (neu co)
                for t in plan[i + 1:]:
                    if t['skill'] == 'place' and t['zone'] == s['zone']:
                        keep = t['object']
                        break
            if s['skill'] == 'pick':            # vat da nam dung zone dich -> bo qua
                j = next_place(plan, i, s['object'])
                if j is not None and plan[j]['skill'] == 'place' \
                        and sk.sc.where(s['object']) == plan[j]['zone']:
                    skip.add(j)
                    log(f'    {s["object"]} da o {plan[j]["zone"]} -> bo qua pick/place')
                    res.append((lb, 'SKIPPED'))
                    log(f'{lb} {"." * max(2, 34 - len(lb))} SKIPPED')
                    continue
            try:
                st = do_step(s, sk, keep)
            except Exception as e:
                log(f'    loi: {e}')
                st = 'FAILED'
            if st not in OK_ST:
                ok = False
        log(f'{lb} {"." * max(2, 34 - len(lb))} {st}')
        res.append((lb, st))
    return ok, res


def fix_text(s):
    """Bo byte loi UTF-8 truoc khi gui LLM."""
    return s.encode('utf-8', 'surrogateescape').decode('utf-8', 'ignore').strip()


def show_state(sc, log):
    for o, w in sc.state().items():
        p = sc.pos[o]
        xy = f' ({p[0]:.2f}, {p[1]:.2f})' if p else ''
        log(f'  {o:12s} {w}{xy}')
    log('  zones: ' + ', '.join(f'{z}={w}' for z, w in sc.zone_state().items()))


def run_cmd(cmd, pl, sk, log=print):
    """Toan bo luong: USER COMMAND -> camera -> LLM -> validator -> executor."""
    cmd = fix_text(cmd)
    rep = {'command': cmd, 'plan': [], 'status': 'TASK FAILED'}
    log(BAR)
    log('USER COMMAND:')
    log(cmd)
    log('')

    if sk.detect_objects() != 'SUCCESS':
        log('CAMERA ERROR: khong lay duoc trang thai moi truong')
        log(BAR)
        return rep
    log('CAMERA:')
    show_state(sk.sc, log)
    log('')

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

    rep['plan'] = plan
    log('LLM PLAN:')
    for i, s in enumerate(plan, 1):
        log(f'{i}. {to_str(s)}' + ('   [auto: validator them vao]' if s.get('note') == 'auto' else ''))
    log('')
    log('JSON: ' + json.dumps({'plan': [{k: v for k, v in s.items() if k != 'note'} for s in plan]}))
    log('')

    log('EXECUTION:')
    ok, res = run(plan, sk, log)
    rep['steps'] = res
    rep['status'] = 'TASK SUCCESS' if ok else 'TASK FAILED'
    log('')
    log(rep['status'])
    if ok and sk.detect_objects() == 'SUCCESS':
        log('CAMERA (sau khi lam):')
        show_state(sk.sc, log)
    log(BAR)
    return rep
