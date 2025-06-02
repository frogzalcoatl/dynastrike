import { Vector2 } from "../../geometry/vector";

export function computeAveragePoint(points: number[]): Vector2 {
	let sumX: number = 0;
	let sumY: number = 0;
	for (let i: number = 0; i < points.length; i += 2) {
		sumX += points[i];
		sumY += points[i + 1];
	}
	const invCount = 1 / points.length * 2;
	return {
		x: sumX * invCount,
		y: sumY * invCount
	};
}