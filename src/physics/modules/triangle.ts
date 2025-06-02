export type Triangle = [number, number, number, number, number, number];

export function isPointInTriangle(pointX: number, pointY: number, triangle: Triangle): boolean {
	const vertex1X: number = triangle[0];
	const vertex1Y: number = triangle[1];
	const vertex2X: number = triangle[2];
	const vertex2Y: number = triangle[3];
	const vertex3X: number = triangle[4];
	const vertex3Y: number = triangle[5];
	const denominator: number = (vertex2Y - vertex3Y) * (vertex1X - vertex3X) + (vertex3X - vertex2X) * (vertex1Y - vertex3Y);
	if (Math.abs(denominator) < 1e-12) {
		return false;
	}
	const inverseDenominator: number = 1 / denominator;
	const barycentricX = ((vertex2Y - vertex3Y) * (pointX - vertex3X) + (vertex3X - vertex2X) * (pointY - vertex3Y)) * inverseDenominator;
	const barycentricY = ((vertex3Y - vertex1Y) * (pointX - vertex3X) + (vertex1X - vertex3X) * (pointY - vertex3Y)) * inverseDenominator;
	return (barycentricX >= 0) && (barycentricY >= 0) && ((barycentricX + barycentricY) <= 1);
}

export function triangulate(points: number[]): Triangle[] {
	const pointCount: number = points.length >> 1;
	if (pointCount < 3) {
		return [];
	}
	const nextIndices: number[] = [];
	const previousIndices: number[] = [];
	for (let i: number = 0; i < pointCount; i++) {
		nextIndices.push((i + 1) % pointCount);
		previousIndices.push((i + pointCount - 1) % pointCount);
	}
	let area: number = 0;
	for (let i: number = 0; i < pointCount; i++) {
		const j: number = (i + 1) % pointCount;
		const currentVertexX: number = points[2 * i];
		const currentVertexY: number = points[2 * i + 1];
		const nextVertexX: number = points[2 * j];
		const nextVertexY: number = points[2 * j + 1];
		area += currentVertexX * nextVertexY - nextVertexX * currentVertexY;
	}
	const isCounterClockWise: boolean = area > 0;
	const triangles: Triangle[] = [];
	let remainingPoints: number = pointCount;
	let currentIndex: number = 0;
	while (remainingPoints > 3) {
		let earFound: boolean = false;
		for (let pass: number = 0; pass < remainingPoints; pass++) {
			const previousIndex: number = previousIndices[currentIndex];
			const nextIndex: number = nextIndices[currentIndex];
			const previousVertexX: number = points[2 * previousIndex];
			const previousVertexY: number = points[2 * previousIndex + 1];
			const currentIndexX: number = points[2 * currentIndex];
			const currentIndexY: number = points[2 * currentIndex + 1];
			const nextIndexX: number = points[2 * nextIndex];
			const nextIndexY: number = points[2 * nextIndex + 1];
			const cross: number = (currentIndexX - previousVertexX) * (nextIndexY - currentIndexY) - (currentIndexY - previousVertexY) * (nextIndexX - currentIndexX);
			if (isCounterClockWise ? cross <= 0 : cross >= 0) {
				currentIndex = nextIndex;
				continue;
			}
			const candidateTriangle: Triangle = [previousVertexX, previousVertexY, currentIndexX, currentIndexY, nextIndexX, nextIndexY];
			let hasInside: boolean = false;
			let walker: number = nextIndices[nextIndex];
			for (let k: number = 0; k < remainingPoints - 3; k++) {
				if (walker === previousIndex) {
					break;
				}
				const pointX: number = points[2 * walker];
				const pointY: number = points[2 * walker + 1];
				if (isPointInTriangle(pointX, pointY, candidateTriangle)) {
					hasInside = true;
					break;
				}
				walker = nextIndices[walker];
			}
			if (hasInside) {
				currentIndex = nextIndex;
				continue;
			}
			triangles.push(candidateTriangle);
			nextIndices[previousIndex] = nextIndex;
			previousIndices[nextIndex] = previousIndex;
			remainingPoints--;
			earFound = true;
			currentIndex = nextIndex;
			break;
		}
		if (!earFound) {
			break;
		}
	}
	if (remainingPoints === 3) {
		const index0: number = currentIndex;
		const index1: number = nextIndices[index0];
		const index2: number = nextIndices[index1];
		triangles.push([points[2 * index0], points[2 * index0 + 1], points[2 * index1], points[2 * index1 + 1], points[2 * index2], points[2 * index2 + 1]]);
	}
	return triangles;
}