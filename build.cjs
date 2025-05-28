const esbuild = require("esbuild");
const args = process.argv.slice(2);

const isWatch = args.includes("--watch");

const commonOptions = {
	entryPoints: ["./src/index.ts"],
	bundle: true,
	sourcemap: true,
	logLevel: "info"
};

const presets = [
	{
		format: "cjs",
		outfile: "dist/dynastrike.cjs.js",
		platform: "node"
	},
	{

		format: "esm",
		outfile: "dist/dynastrike.esm.js",
		platform: "neutral",
	},
	{
		minify: true,
		format: "iife",
		outfile: "dist/dynastrike.global.js",
		globalName: "Dynastrike",
		platform: "browser"
	}
];

for (const preset of presets) {
	const options = { ...commonOptions, ...preset };
	(async () => {
		const ctx = await esbuild.context(options);
		if (isWatch) {
			await ctx.watch();
			console.log(`Watching to outfile: ${options.outfile}`);
		} else {
			await ctx.rebuild();
			await ctx.dispose();
			console.log(`Built outfile: ${options.outfile}`);
		}
	})();
}