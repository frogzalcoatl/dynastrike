export function generatePolygon(centerX: number, centerY: number, radius: number, sides: number): number[] {
	const output: number[] = [];
	for (let i: number = 0; i < sides; i++) {
		const angle: number = (2 * Math.PI * i) / sides - Math.PI / 2;
		output.push(centerX + radius * Math.cos(angle), centerY + radius * Math.sin(angle));
	}
	return output;
}

export function generateStar(centerX: number, centerY: number, size: number, outerRadius: number, innerRadius: number, points: number): number[] {
	const output: number[] = [];
	const scaledOuter = outerRadius * size;
	const scaledInner = innerRadius * size;
	const step: number = Math.PI / points;
	for (let i: number = 0; i < points * 2; i++) {
		const radius: number = i % 2 === 0 ? scaledOuter : scaledInner;
		const angle: number = i * step - Math.PI / 2;
		output.push(centerX + radius * Math.cos(angle), centerY + radius * Math.sin(angle));
	}
	return output;
}

export function generateHeart(centerX: number, centerY: number, size: number, resolution: number): number[] {
	const output: number[] = [];
	for (let i: number = 0; i < resolution; i++) {
		const tan: number = (Math.PI * 2 * i) / resolution;
		const pointX: number = size * 16 * Math.pow(Math.sin(tan), 3);
		const pointY: number = -size * (13 * Math.cos(tan) - 5 * Math.cos(2 * tan) - 2 * Math.cos(3 * tan) - Math.cos(4 * tan));
		output.push(centerX + pointX, centerY + pointY);
	}
	return output;
}

export function generateEllipse(centerX: number, centerY: number, radiusX: number, radiusY: number, resolution: number): number[] {
	const output: number[] = [];
	for (let i: number = 0; i < resolution; i++) {
		const angle: number = (2 * Math.PI * i) / resolution;
		output.push(centerX + radiusX * Math.cos(angle), centerY + radiusY * Math.sin(angle));
	}
	return output;
}

export function generateRectangle(centerX: number, centerY: number, width: number, height: number): number[] {
	const hw = width / 2;
	const hh = height / 2;
	return [
		centerX - hw, centerY - hh,
		centerX + hw, centerY - hh,
		centerX + hw, centerY + hh,
		centerX - hw, centerY + hh
	];
}