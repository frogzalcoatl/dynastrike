import { ClosestPoint, ContactPoints, ProjectionRange } from "../types";
import { projectPointOnEdge } from "./misc";

export function isPointInPolygon(pointX: number, pointY: number, points: number[]): boolean {
	let isInside: boolean = false;
	const pointCount: number = points.length;
	let previousX: number = points[pointCount - 2];
	let previousY: number = points[pointCount - 1];
	for (let i: number = 0; i < pointCount; i += 2) {
		const currentX: number = points[i];
		const currentY: number = points[i + 1];
		if (pointX === currentX && pointY === currentY) {
			return true;
		}
		if ((currentY > pointY) !== (previousY > pointY) && pointX < currentX + (previousX - currentX) * (pointY - currentY) / (previousY - currentY)) {
			isInside = !isInside;
		}
		previousX = currentX;
		previousY = currentY;
	}
	return isInside;
}

export function findClosestPointOnPolygonToCircle(points: number[], centerX: number, centerY: number): ClosestPoint {
	const lastVertexIndexOffset: number = points.length - 2;
	let closestPointX: number = 0;
	let closestPointY: number = 0;
	let minimumDistanceSquared: number = Infinity;
	for (let i: number = 0; i < lastVertexIndexOffset; i += 2) {
		const edgeX: number = points[i];
		const edgeY: number = points[i + 1];
		const edgeDeltaX: number = points[i + 2] - edgeX;
		const edgeDeltaY: number = points[i + 3] - edgeY;
		const edgeLengthSquared: number = edgeDeltaX * edgeDeltaX + edgeDeltaY * edgeDeltaY;
		const projectionFactor: number = edgeLengthSquared === 0 ? 0 : ((centerX - edgeX) * edgeDeltaX + (centerY - edgeY) * edgeDeltaY) / edgeLengthSquared;
		const clampedProjectionFactor: number = projectionFactor < 0 ? 0 : projectionFactor > 1 ? 1 : projectionFactor;
		const projectedX: number = edgeX + edgeDeltaX * clampedProjectionFactor;
		const projectedY: number = edgeY + edgeDeltaY * clampedProjectionFactor;
		const deltaX: number = projectedX - centerX;
		const deltaY: number = projectedY - centerY;
		const currentDistanceSquared: number = deltaX * deltaX + deltaY * deltaY;
		if (currentDistanceSquared < minimumDistanceSquared) {
			minimumDistanceSquared = currentDistanceSquared;
			closestPointX = projectedX;
			closestPointY = projectedY;
		}
	}
	const edgeStartX: number = points[lastVertexIndexOffset];
	const edgeStartY: number = points[lastVertexIndexOffset + 1];
	const edgeDeltaX: number = points[0] - edgeStartX;
	const edgeDeltaY: number = points[1] - edgeStartY;
	const edgeLengthSquared: number = edgeDeltaX * edgeDeltaX + edgeDeltaY * edgeDeltaY;
	const projectionFactor: number = edgeLengthSquared === 0 ? 0 : ((centerX - edgeStartX) * edgeDeltaX + (centerY - edgeStartY) * edgeDeltaY) / edgeLengthSquared;
	const clampedProjectionFactor: number = projectionFactor < 0 ? 0 : projectionFactor > 1 ? 1 : projectionFactor;
	const projectedX: number = edgeStartX + edgeDeltaX * clampedProjectionFactor;
	const projectedY: number = edgeStartY + edgeDeltaY * clampedProjectionFactor;
	const deltaX: number = projectedX - centerX;
	const deltaY: number = projectedY - centerY;
	const currentDistanceSquared: number = deltaX * deltaX + deltaY * deltaY;
	if (currentDistanceSquared < minimumDistanceSquared) {
		minimumDistanceSquared = currentDistanceSquared;
		closestPointX = projectedX;
		closestPointY = projectedY;
	}
	return {
		x: closestPointX,
		y: closestPointY,
		distanceSquared: minimumDistanceSquared
	};
}

export function addPolygonAxes(points: number[], output: number[]): void {
	const pointCount: number = points.length;
	for (let i: number = 0; i < pointCount; i += 2) {
		const normalX: number = -(points[(i + 3) % pointCount] - points[i + 1]);
		const normalY: number = points[(i + 2) % pointCount] - points[i];
		const normalLength: number = Math.sqrt(normalX * normalX + normalY * normalY);
		if (normalLength > 1e-9) {
			output.push(normalX / normalLength, normalY / normalLength);
		}
	}
}

export function projectPolygonOntoAxis(points: number[], axisX: number, axisY: number): ProjectionRange {
	let minimumProjection: number = Infinity;
	let maximumProjection: number = -Infinity;
	for (let i: number = 0; i < points.length; i += 2) {
		const currentProjection: number = points[i] * axisX + points[i + 1] * axisY;
		if (currentProjection < minimumProjection) {
			minimumProjection = currentProjection;
		}
		if (currentProjection > maximumProjection) {
			maximumProjection = currentProjection;
		}
	}
	return {
		minimum: minimumProjection,
		maximum: maximumProjection
	};
}

export function findContactPoints(instancePoints: number[], otherPoints: number[]): ContactPoints {
	const instancePointCount: number = instancePoints.length;
	const otherPointCount: number = otherPoints.length;
	const instanceVertexCount: number = instancePointCount >> 1;
	const otherVertexCount: number = otherPointCount >> 1;
	let instanceBest: number = -Infinity;
	let instanceSumX: number = 0;
	let instanceSumY: number = 0;
	let instanceCount: number = 0;
	for (let i: number = 0; i < instancePointCount; i += 2) {
		const pointX: number = instancePoints[i];
		const pointY: number = instancePoints[i + 1];
		if (!isPointInPolygon(pointX, pointY, otherPoints)) {
			continue;
		}
		let minimumDistance: number = Infinity;
		for (let j: number = 0; j < otherPointCount; j += 2) {
			const k: number = (j + 2) % otherPointCount;
			const distanceSquared: number = projectPointOnEdge(pointX, pointY, otherPoints[j], otherPoints[j + 1], otherPoints[k], otherPoints[k + 1]).distanceSquared;
			if (distanceSquared < minimumDistance) {
				minimumDistance = distanceSquared;
			}
		}
		if (minimumDistance > instanceBest) {
			instanceBest = minimumDistance;
			instanceSumX = pointX;
			instanceSumY = pointY;
			instanceCount = 1;
		} else if (minimumDistance === instanceBest) {
			instanceSumX += pointX;
			instanceSumY += pointY;
			instanceCount++;
		}
	}
	let otherBest: number = -Infinity;
	let otherSumX: number = 0;
	let otherSumY: number = 0;
	let otherCount: number = 0;
	for (let i: number = 0; i < otherPointCount; i += 2) {
		const pointX: number = otherPoints[i];
		const pointY: number = otherPoints[i + 1];
		if (!isPointInPolygon(pointX, pointY, instancePoints)) {
			continue;
		}
		let minimumDistance = Infinity;
		for (let j: number = 0; j < instancePointCount; j += 2) {
			const k: number = (j + 2) % instancePointCount;
			const distanceSquared: number = projectPointOnEdge(pointX, pointY, instancePoints[j], instancePoints[j + 1], instancePoints[k], instancePoints[k + 1]).distanceSquared;
			if (distanceSquared < minimumDistance) {
				minimumDistance = distanceSquared;
			}
		}
		if (minimumDistance > otherBest) {
			otherBest = minimumDistance;
			otherSumX = pointX;
			otherSumY = pointY;
			otherCount = 1;
		} else if (minimumDistance === otherBest) {
			otherSumX += pointX;
			otherSumY += pointY;
			otherCount++;
		}
	}
	let finalSumX: number = instanceSumX;
	let finalSumY: number = instanceSumY;
	let finalCount: number = instanceCount;
	let finalBest: number = instanceBest;
	if (otherCount > 0 && (instanceCount === 0 || otherBest > instanceBest)) {
		finalSumX = otherSumX;
		finalSumY = otherSumY;
		finalCount = otherCount;
		finalBest = otherBest;
	}
	if (finalCount === 0) {
		let sumIX: number = 0;
		let sumIY: number = 0;
		let sumOX: number = 0;
		let sumOY: number = 0;
		for (let i: number = 0; i < instancePointCount; i += 2) {
			sumIX += instancePoints[i];
			sumIY += instancePoints[i + 1];
		}
		for (let i: number = 0; i < otherPointCount; i += 2) {
			sumOX += otherPoints[i];
			sumOY += otherPoints[i + 1];
		}
		return {
			x: (sumIX / instanceVertexCount + sumOX / otherVertexCount) * 0.5,
			y: (sumIY / instanceVertexCount + sumOY / otherVertexCount) * 0.5,
			count: 1
		};
	}
	return {
		x: finalSumX / finalCount,
		y: finalSumY / finalCount,
		count: finalCount
	};
}

/*export function isPointInTriangle(pointX: number, pointY: number, triangle: Triangle): boolean {
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
}*/