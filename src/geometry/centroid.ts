import { Vector2 } from "..";

export function computeCentroid(points: number[]): Vector2 {
	let centerX: number = 0;
	let centerY: number = 0;
	let totalCross: number = 0;
	const length = points.length;
	for (let i: number = 0, j: number = length - 2; i < length; j = i, i += 2) {
		const currentX: number = points[i];
		const currentY: number = points[i + 1];
		const previousX: number = points[j];
		const previousY: number = points[j + 1];
		const cross: number = previousX * currentY - currentX * previousY;
		totalCross += cross;
		centerX += (previousX + currentX) * cross;
		centerY += (previousY + currentY) * cross;
	}
	if (totalCross === 0) {
		return {
			x: 0,
			y: 0
		};
	}
	const factor: number = 1 / (3 * totalCross);
	return {
		x: centerX * factor,
		y: centerY * factor
	};
}