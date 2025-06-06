import { Triangle, Vector2 } from "../types";

export function computeAveragePoint(points: number[]): Vector2 {
	const pointCount: number = points.length;
	let sumX: number = 0;
	let sumY: number = 0;
	for (let i: number = 0; i < pointCount; i += 2) {
		sumX += points[i];
		sumY += points[i + 1];
	}
	const inversePointCount = 1 / pointCount * 2;
	return {
		x: sumX * inversePointCount,
		y: sumY * inversePointCount
	};
}

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


export function isPointInPolygon(pointX: number, pointY: number, points: number[]): boolean {
	let isInside: boolean = false;
	const pointCount: number = points.length;
	let previousPointX: number = points[pointCount - 2];
	let previousPointY: number = points[pointCount - 1];
	for (let i: number = 0; i < pointCount; i += 2) {
		const currentPointX: number = points[i];
		const currentPointY: number = points[i + 1];
		if (pointX === currentPointX && pointY === currentPointY) {
			return true;
		}
		if ((currentPointY > pointY) !== (previousPointY > pointY) && pointX < currentPointX + (previousPointX - currentPointX) * (pointY - currentPointY) / (previousPointY - currentPointY)) {
			isInside = !isInside;
		}
		previousPointX = currentPointX;
		previousPointY = currentPointY;
	}
	return isInside;
}

export function isPointInTriangle(pointX: number, pointY: number, triangle: Triangle): boolean {
	const vertex1X: number = triangle[0];
	const vertex1Y: number = triangle[1];
	const vertex2X: number = triangle[2];
	const vertex2Y: number = triangle[3];
	const vertex3X: number = triangle[4];
	const vertex3Y: number = triangle[5];
	const vertex2YDifferenceToVertex3Y: number = vertex2Y - vertex3Y;
	const vertex3XDifferenceToVertex2X: number = vertex3X - vertex2X;
	const vertex1XDifferenceToVertex3X: number = vertex1X - vertex3X;
	const vertex1YDifferenceToVertex3Y: number = vertex1Y - vertex3Y;
	const denominator: number = vertex2YDifferenceToVertex3Y * vertex1XDifferenceToVertex3X + vertex3XDifferenceToVertex2X * vertex1YDifferenceToVertex3Y;
	if (denominator === 0) {
		return false;
	}
	const inverseDenominator: number = 1 / denominator;
	const differenceX: number = pointX - vertex3X;
	const differenceY: number = pointY - vertex3Y;
	const barycentric1: number = (vertex2YDifferenceToVertex3Y * differenceX + vertex3XDifferenceToVertex2X * differenceY) * inverseDenominator;
	const Vertex3YDifferenceToVertex1Y: number = vertex3Y - vertex1Y;
	const barycentric2: number = (Vertex3YDifferenceToVertex1Y * differenceX + vertex1XDifferenceToVertex3X * differenceY) * inverseDenominator;
	return barycentric1 >= 0 && barycentric2 >= 0 && barycentric1 + barycentric2 <= 1;
}

export function triangulate(points: number[]): Triangle[] {
	const totalCoords: number = points.length;
	const vertexCount: number = totalCoords >> 1;
	const nextVertexIndex: number[] = [];
	const previousVertexIndex: number[] = [];
	for (let i = 0; i < vertexCount; i++) {
		nextVertexIndex.push(i + 1 === vertexCount ? 0 : i + 1);
		previousVertexIndex.push(i === 0 ? vertexCount - 1 : i - 1);
	}
	let signedArea: number = 0;
	for (let i = 0; i < vertexCount; i++) {
		const j: number = nextVertexIndex[i];
		const currentVertexX: number = points[(i << 1)];
		const currentVertexY: number = points[(i << 1) + 1];
		const nextVertexX: number = points[(j << 1)];
		const nextVertexY: number = points[(j << 1) + 1];
		signedArea += currentVertexX * nextVertexY - nextVertexX * currentVertexY;
	}
	const isCounterClockwise: boolean = signedArea > 0;
	const triangles: Triangle[] = [];
	let remainingVertices: number = vertexCount;
	let currentVertex: number = 0;
	while (remainingVertices > 3) {
		let earFound: boolean = false;
		for (let pass = 0; pass < remainingVertices; pass++) {
			const previousVertex: number = previousVertexIndex[currentVertex];
			const nextVertex: number = nextVertexIndex[currentVertex];
			const previousVertexX: number = points[(previousVertex << 1)];
			const previousVertexY: number = points[(previousVertex << 1) + 1];
			const currentVertexX: number = points[(currentVertex << 1)];
			const currentVertexY: number = points[(currentVertex << 1) + 1];
			const nextVertexX: number = points[(nextVertex << 1)];
			const nextVertexY: number = points[(nextVertex << 1) + 1];
			const crossProduct: number = (currentVertexX - previousVertexX) * (nextVertexY - currentVertexY) - (currentVertexY - previousVertexY) * (nextVertexX - currentVertexX);
			if (isCounterClockwise ? crossProduct <= 0 : crossProduct >= 0) {
				currentVertex = nextVertex;
				continue;
			}
			const currentYminusNextY: number = currentVertexY - nextVertexY;
			const nextXminusCurrentX: number = nextVertexX - currentVertexX;
			const previousXminusNextX: number = previousVertexX - nextVertexX;
			const previousYminusNextY: number = previousVertexY - nextVertexY;
			const denominatorEar: number = currentYminusNextY * previousXminusNextX + nextXminusCurrentX * previousYminusNextY;
			if (denominatorEar === 0) {
				currentVertex = nextVertex;
				continue;
			}
			const invDenominatorEar: number = 1 / denominatorEar;
			const nextYminusPrevY: number = nextVertexY - previousVertexY;
			let walker: number = nextVertexIndex[nextVertex];
			let pointInsideCandidate: boolean = false;
			for (let k = remainingVertices - 3; k > 0; k--) {
				if (walker === previousVertex) {
					break;
				}
				const testX: number = points[(walker << 1)];
				const testY: number = points[(walker << 1) + 1];
				const testXminusNextX: number = testX - nextVertexX;
				const testYminusNextY: number = testY - nextVertexY;
				const barycentric1: number = (currentYminusNextY * testXminusNextX + nextXminusCurrentX * testYminusNextY) * invDenominatorEar;
				if (barycentric1 < 0) {
					walker = nextVertexIndex[walker];
					continue;
				}
				const barycentric2: number =
					(nextYminusPrevY * testXminusNextX +
						previousXminusNextX * testYminusNextY) *
					invDenominatorEar;
				if (barycentric2 < 0 || barycentric1 + barycentric2 > 1) {
					walker = nextVertexIndex[walker];
					continue;
				}
				pointInsideCandidate = true;
				break;
			}
			if (pointInsideCandidate) {
				currentVertex = nextVertex;
				continue;
			}
			triangles.push([previousVertexX, previousVertexY, currentVertexX, currentVertexY, nextVertexX, nextVertexY,]);
			nextVertexIndex[previousVertex] = nextVertex;
			previousVertexIndex[nextVertex] = previousVertex;
			remainingVertices--;
			earFound = true;
			currentVertex = nextVertex;
			break;
		}
		if (!earFound) {
			break;
		}
	}
	if (remainingVertices === 3) {
		const i0: number = currentVertex;
		const i1: number = nextVertexIndex[i0];
		const i2: number = nextVertexIndex[i1];
		triangles.push([points[(i0 << 1)], points[(i0 << 1) + 1], points[(i1 << 1)], points[(i1 << 1) + 1], points[(i2 << 1)], points[(i2 << 1) + 1]]);
	}

	return triangles;
}