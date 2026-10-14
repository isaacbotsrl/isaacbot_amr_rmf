// Copyright 2026 IsaacBot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Helpers shared by the robots page (index.html) and a robot's page (robot.html).

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

async function api(method, path, body) {
  const res = await fetch(path, {method, headers: {'Content-Type': 'application/json'},
                                 body: body ? JSON.stringify(body) : undefined});
  const data = await res.json().catch(() => ({error: `HTTP ${res.status}`}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function ago(seconds) {
  const d = Date.now() / 1000 - seconds;
  if (d < 90) return 'just now';
  if (d < 5400) return `${Math.round(d / 60)} min ago`;
  if (d < 172800) return `${Math.round(d / 3600)} h ago`;
  return new Date(seconds * 1000).toLocaleDateString();
}

// Sets innerHTML only when it changed: the 3 s refresh would otherwise wipe a text selection or
// an open <select>.
const rendered = {};
function setHtml(id, html) {
  if (rendered[id] !== html) $(id).innerHTML = rendered[id] = html;
}

// The header's RMF status, from /api/status.
function renderRmf(status) {
  const rmf = status.rmf || {}, wanted = status.active || rmf.site;
  let cls = 'bad', text = 'RMF is not running';
  if (rmf.state === 'running' && (!status.active || rmf.site === status.active)) {
    cls = 'ok'; text = `RMF runs site ${rmf.site} (since ${ago(rmf.started)})`;
  } else if (wanted) {
    cls = 'warn'; text = `RMF is switching to site ${wanted}…`;
  }
  if (rmf.error) { cls = 'bad'; text += ` — ${rmf.error}`; }
  setHtml('rmf', `<span class="dot ${cls}"></span><span>${esc(text)}</span>`);
}

// A robot's live state from the fleet adapter (null: RMF does not know it yet).
function robotState(r) {
  const live = r.live;
  if (!live) return '<span class="status"><span class="dot"></span>not in RMF yet</span>';
  const online = live.connection === 'ONLINE';
  const cls = !online ? 'bad' : live.in_fleet === false || live.blocked_reason ? 'warn' : 'ok';
  const parts = [online ? (live.operating_mode || 'online') : live.connection.toLowerCase()];
  if (live.battery_soc !== null && live.battery_soc !== undefined) parts.push(`${Math.round(live.battery_soc * 100)} %`);
  if (online && !live.localized) parts.push('not localized');
  const why = live.blocked_reason ? `<div class="muted">${esc(live.blocked_reason)}</div>` : '';
  return `<span class="status"><span class="dot ${cls}"></span>${esc(parts.join(' · '))}</span>${why}`;
}

// The balena variables after adding a robot or rotating its password (shown once).
function showCredentials(target, r, what) {
  const vars = Object.entries(r.balena).map(([k, v]) => `${k}=${v}`).join('\n');
  const once = r.password ? '<b>The password is shown only now</b>; ' : '';
  $(target).innerHTML = `<div class="message ok"><b>${esc(r.robot.name)}</b> ${what}.
    ${once}set these balenaCloud device variables on the isaacbot (each change restarts its VDA 5050 container):
    <div class="cmd"><pre>${esc(vars)}</pre><button data-copy>Copy</button></div></div>`;
}

async function copyText(text) {
  if (window.isSecureContext && navigator.clipboard) return navigator.clipboard.writeText(text);
  // Plain http on the LAN or VPN address has no navigator.clipboard.
  const t = document.createElement('textarea');
  t.value = text; t.style.position = 'fixed'; t.style.opacity = '0';
  document.body.appendChild(t); t.select();
  const ok = document.execCommand('copy');
  t.remove();
  if (!ok) throw new Error('copy refused');
}

async function copyCommand(button) {
  const pre = button.previousElementSibling;
  try {
    await copyText(pre.textContent);
    button.textContent = 'Copied ✓';
  } catch (e) {
    getSelection().selectAllChildren(pre);
    button.textContent = 'Press Ctrl+C';
  }
  setTimeout(() => { button.textContent = 'Copy'; }, 1500);
}

// Click handling: each page passes its own handlers; Copy buttons work everywhere.
function onButtons(handlers) {
  document.addEventListener('click', async (event) => {
    const b = event.target.closest('button');
    if (!b) return;
    try {
      if ('copy' in b.dataset) return await copyCommand(b);
      for (const [key, handler] of Object.entries(handlers)) {
        if (b.id === key || b.dataset[key] !== undefined) return await handler(b, b.dataset[key]);
      }
    } catch (e) { alert(e.message); b.disabled = false; }
  });
}
