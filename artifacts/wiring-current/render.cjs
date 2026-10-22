// Render the vector masters; PDF creation is handled separately by build_pdf.py.
const fs = require('fs');
const path = require('path');
const sharp = require('C:/Users/shado/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
(async () => {
  for (const name of fs.readdirSync(__dirname).filter(name => /^0[0-7]-.*\.svg$/.test(name)).sort()) {
    const target = path.join(__dirname, name.replace(/\.svg$/, '.png'));
    const width = name.startsWith('00-') ? 4800 : 2700;
    // Render before replacing the destination, so previews never read partial PNGs.
    const rendered = await sharp(path.join(__dirname, name), { density: 144 }).resize({ width }).png().toBuffer();
    const pending = target + '.rendering.tmp';
    fs.writeFileSync(pending, rendered);
    fs.renameSync(pending, target);
    console.log(path.basename(target));
  }
})().catch(error => { console.error(error); process.exitCode = 1; });
