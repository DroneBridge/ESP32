import { copyFile, mkdir, readdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { inlineSource } from "inline-source";

const frontendDirectory = path.dirname(fileURLToPath(import.meta.url));
const sourcePath = path.join(frontendDirectory, "index.html");
const buildDirectory = path.join(frontendDirectory, "build");
const outputPath = path.join(buildDirectory, "index.html");

await mkdir(buildDirectory, { recursive: true });

const inlinedHtml = await inlineSource(sourcePath, {
    compress: true,
    rootpath: frontendDirectory,
});

await writeFile(outputPath, inlinedHtml, "utf8");

const assetNames = (await readdir(frontendDirectory)).filter((name) =>
    /\.(?:png|ico)$/i.test(name),
);

await Promise.all(
    assetNames.map((assetName) =>
        copyFile(
            path.join(frontendDirectory, assetName),
            path.join(buildDirectory, assetName),
        ),
    ),
);
