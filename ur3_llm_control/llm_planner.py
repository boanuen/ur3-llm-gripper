"""LLM Planner: cau lenh -> JSON plan (qua 9Router). LLM chi chon skill."""
import json
import os
import re
import time
import urllib.error
import urllib.request

from .scene import cfg_path
from .student import assign, get_p
from .task_validator import SKILLS, check


def make_prompt(sc, name, sid):
    # doc config/prompt.txt roi dien thong tin vao
    with open(cfg_path('prompt.txt'), encoding='utf-8') as f:
        tpl = f.read()
    sk = []
    for k in SKILLS:
        sk.append(f"  {k}({', '.join(SKILLS[k])})")
    ob = []
    for o in sc.objs:
        ob.append(f'  {o}')
    zs = []
    for z in sc.zones:
        zs.append(f'  {z}   ("Zone {z[-1].upper()}")')
    asg = []
    a = assign(sid)
    for z in a:
        asg.append(f'  {a[z]} -> {z}')
    return tpl.format(skills='\n'.join(sk), objects='\n'.join(ob), zones='\n'.join(zs),
                      name=name, sid=sid, p=get_p(sid), assign='\n'.join(asg))


def ask_one(url, key, model, msgs):
    # gui 1 request, tra ve text
    body = {'model': model, 'messages': msgs, 'temperature': 0}
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method='POST',
                                 headers={'Content-Type': 'application/json',
                                          'Authorization': 'Bearer ' + key})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            txt = r.read().decode()
        try:
            data = json.loads(txt)
        except ValueError:
            raise RuntimeError('9Router tra ve noi dung rong / khong phai JSON (model loi?)')
        return data['choices'][0]['message']['content'] or ''
    except urllib.error.HTTPError as e:
        raise RuntimeError(f'HTTP {e.code}: {e.read().decode(errors="replace")[:200]}')
    except Exception as e:
        raise RuntimeError(str(e))


def ask(cfg, msgs):
    # moi model thu 3 lan, loi thi doi sang backup_model
    url = cfg['base_url'].rstrip('/') + '/chat/completions'
    key = os.environ.get('NINE_KEY') or cfg.get('api_key', '')
    models = [cfg['model']]
    if cfg.get('backup_model'):
        models.append(cfg['backup_model'])
    err = ''
    for m in models:
        for _ in range(3):
            try:
                return ask_one(url, key, m, msgs)
            except RuntimeError as e:
                err = f'{m}: {e}'
                print(f'  [LLM] loi {err} -> thu lai', flush=True)
                time.sleep(2)
    raise RuntimeError(f'loi goi 9Router ({url}): {err}')


def get_json(txt):
    # lay object JSON dau tien (bo <think>, ```json)
    txt = re.sub(r'<think>.*?</think>', '', txt, flags=re.S)
    i = txt.find('{')
    if i < 0:
        raise ValueError('khong co JSON')
    obj, _ = json.JSONDecoder().raw_decode(txt[i:])
    return obj


class Planner:
    def __init__(self, cfg, sc, name, sid, ask_fn=ask):
        self.cfg = cfg
        self.sc = sc
        self.sys = make_prompt(sc, name, sid)
        self.ask = ask_fn          # thay duoc khi test

    def plan(self, cmd):
        """Tra ve (plan, errs, cau tra loi cua LLM)."""
        # trang thai camera
        st = self.sc.state()
        a = []
        for o in st:
            a.append(f'{o}: {st[o]}')
        zs = self.sc.zone_state()
        b = []
        for z in zs:
            b.append(f'{z}: {zs[z]}')
        msgs = [{'role': 'system', 'content': self.sys},
                {'role': 'user', 'content':
                 f"Camera state: {', '.join(a)}.\nZones: {', '.join(b)}.\nCommand: {cmd}"}]

        plan = []
        errs = []
        ans = ''
        for _ in range(int(self.cfg.get('tries', 2))):
            ans = self.ask(self.cfg, msgs)
            try:
                raw = get_json(ans)
            except ValueError as e:
                raw = None
                errs = [f'LLM tra ve khong phai JSON: {e}']
            if raw is not None:
                plan, errs = check(raw, self.sc)
            if not errs:
                return plan, [], ans
            # LLM tu choi -> khong hoi lai
            if isinstance(raw, dict) and raw.get('plan') == []:
                break
            # bao loi va hoi lai
            msgs.append({'role': 'assistant', 'content': ans})
            msgs.append({'role': 'user', 'content':
                         'Your plan was rejected: ' + '; '.join(errs) +
                         '. Return a corrected JSON plan. Do not change the object or zone '
                         'the user asked for; if impossible return {"plan": [], "error": "..."}.'})
        return plan, errs, ans
