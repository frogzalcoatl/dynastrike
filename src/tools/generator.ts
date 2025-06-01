export function generatePolygon(centerX: number, centerY: number, radius: number, sides: number): number[] {
	const output: number[] = [];
	for (let i: number = 0; i < sides; i++) {
		const angle: number = (2 * Math.PI * i) / sides - Math.PI / 2;
		output.push(centerX + radius * Math.cos(angle), centerY + radius * Math.sin(angle));
	}
	return output;
}

export function generateStar(centerX: number, centerY: number, outerRadius: number, innerRadius: number, points: number[]): number[] {
	const output: number[] = [];
	const numPoints: number = points.length;
	const step: number = Math.PI / numPoints;
	for (let i: number = 0; i < numPoints * 2; i++) {
		const radius: number = i % 2 === 0 ? outerRadius : innerRadius;
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

export function generateArrow(centerX: number, centerY: number, width: number, height: number, headHeightRatio: number): number[] {
	const halfWidth: number = width / 2;
	const halfHeight: number = height / 2;
	const headHeight: number = height * headHeightRatio;
	const shaftHeight: number = height - headHeight;
	return [
		centerX - halfWidth, centerY + halfHeight,
		centerX + halfWidth, centerY + halfHeight,
		centerX + halfWidth, centerY + halfHeight - shaftHeight,
		centerX + halfWidth / 2, centerY + halfHeight - shaftHeight,
		centerX + 0, centerY - halfHeight,
		centerX - halfWidth / 2, centerY + halfHeight - shaftHeight,
		centerX - halfWidth, centerY + halfHeight - shaftHeight
	];
}

export function generateCrescent(centerX: number, centerY: number, outerRadius: number, innerRadius: number, angleOffset: number = 0, resolution: number): number[] {
	const output: number[] = [];
	const startAngle: number = angleOffset;
	const endAngle: number = angleOffset + Math.PI * 2;
	for (let i: number = 0; i <= resolution; i++) {
		const angle: number = startAngle + (i / resolution) * (endAngle - startAngle);
		output.push(centerX + outerRadius * Math.cos(angle), centerY + outerRadius * Math.sin(angle));
	}
	for (let i: number = resolution; i >= 0; i--) {
		const angle: number = startAngle + (i / resolution) * (endAngle - startAngle);
		output.push(centerX + innerRadius * Math.cos(angle + Math.PI * 0.3), centerY + innerRadius * Math.sin(angle + Math.PI * 0.3));
	}
	return output;
}