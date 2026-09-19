// Render the vector masters without changing their electrical content.
const fs = require('fs');
const path = require('path');
const sharp = require('C:/Users/shado/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
(async () => {
  for (const name of fs.readdirSync(__dirname).filter(name => name.endsWith('.svg'))) {
    await sharp(path.join(__dirname, name)).png().toFile(path.join(__dirname, name.replace(/\.svg$/, '.png')));
    console.log(name.replace(/\.svg$/, '.png'));
  }
})();
