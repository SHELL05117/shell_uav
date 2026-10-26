// Render the Markdown guide with the same relative image links, for offline reading.
const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
(async () => {
  const [source, destination, modules] = process.argv.slice(2);
  if (!source || !destination || !modules) throw new Error('Usage: node render_monitor_guide.cjs SOURCE DEST NODE_MODULES');
  const { marked } = await import(pathToFileURL(require.resolve('marked', { paths: [modules] })).href);
  const markdown = fs.readFileSync(source, 'utf8');
  const body = marked.parse(markdown);
  const contents = [];
  let counter = 0;
  const withAnchors = body.replace(/<h2>(.*?)<\/h2>/g, (_, title) => {
    const id = `section-${++counter}`;
    contents.push(`<a href="#${id}">${title}</a>`);
    return `<h2 id="${id}">${title}</h2>`;
  });
  const heading = withAnchors.match(/<h1>[\s\S]*?<\/h1>/)?.[0] || '';
  const content = withAnchors.replace(heading, '');
  fs.writeFileSync(destination, `<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Lichtblick 连接与常用功能教学</title><style>
  :root{color-scheme:light}*{box-sizing:border-box}body{margin:0;color:#1b2838;background:#f3f5f8;font:17px/1.8 "Microsoft YaHei",sans-serif}main{max-width:1120px;margin:24px auto;padding:32px 48px;background:white;border-radius:12px}h1{font-size:30px;line-height:1.5}h2{margin-top:48px;padding-top:16px;border-top:2px solid #d6e1ee;scroll-margin-top:16px;font-size:25px}h3{font-size:20px}a{color:#145dac}img{display:block;max-width:100%;height:auto;margin:20px auto;border:1px solid #cbd5e1;border-radius:6px}table{width:100%;border-collapse:collapse;margin:20px 0;font-size:15px}td,th{border:1px solid #d5dde8;padding:9px 12px;text-align:left;vertical-align:top}th{background:#eef4fc}code{background:#edf2f7;padding:2px 4px;overflow-wrap:anywhere}pre{background:#152334;color:#e7f0ff;padding:18px;border-radius:7px;overflow:auto;line-height:1.55}pre code{background:transparent;padding:0;overflow-wrap:normal}nav{display:grid;gap:4px;background:#f1f6fc;padding:20px;border-radius:8px}nav a{text-decoration:none}li{margin:6px 0}.note{background:#fff2d8;padding:12px 18px;border-left:4px solid #d98c16;font-size:15px}@media(max-width:700px){main{margin:0;padding:20px}table{font-size:13px}}@media print{body{background:white;font-size:11pt}main{padding:0;margin:0}nav{display:none}h2,h3{break-after:avoid}img,table,pre{break-inside:avoid}a{color:inherit}}
  </style><main>${heading}<p class="note">离线图文教程 · SHELL · 实际操作 2026-10-27 凌晨，工程归属 2026-10-26。实时／断线／回放边界见正文。</p><nav>${contents.join('\n')}</nav>${content}</main></html>`, 'utf8');
  const links = [...markdown.matchAll(/!?\[[^\]]*\]\(([^)]+)\)/g)].map(match => match[1]);
  const missing = links.filter(link => !/^(https?:|#)/.test(link) && !fs.existsSync(path.resolve(path.dirname(source), link)));
  console.log(JSON.stringify({ destination, sections: counter, images: (markdown.match(/!\[/g) || []).length, missing }, null, 2));
  if (missing.length) process.exitCode = 1;
})();
