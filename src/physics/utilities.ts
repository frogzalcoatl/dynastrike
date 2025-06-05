import { Vector2 } from "../geometry/vector";

export interface SATResult {
	penetration: number;
	normalX: number;
	normalY: number;
	collided: boolean;
}

export interface ContactResult {
	penetration: number;
	normalX: number;
	normalY: number;
}

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

export function computePenetrationAndNormal(circleX: number, circleY: number, contactX: number, contactY: number, radius: number, polygonCenterX: number, polygonCenterY: number, isInside: boolean, distanceSquared: number, radiusSquared: number): ContactResult {
	if (!isInside && distanceSquared >= radiusSquared) {
		return {
			penetration: 0,
			normalX: 0,
			normalY: 0
		};
	}
	const distance: number = Math.sqrt(distanceSquared);
	const penetration: number = radius - distance;
	if (penetration <= 0) {
		return {
			penetration: 0,
			normalX: 0,
			normalY: 0
		};
	}
	let normalX: number = 0;
	let normalY: number = 0;
	const sign: number = isInside ? -1 : 1;
	if (distance < 1e-18) {
		const distanceX: number = circleX - polygonCenterX;
		const distanceY: number = circleY - polygonCenterY;
		const lengthSquared: number = distanceX * distanceX + distanceY * distanceY;
		if (lengthSquared < 1e-18) {
			normalX = 0;
			normalY = sign;
		} else {
			const inverseLength = 1 / Math.sqrt(lengthSquared);
			normalX = distanceX * inverseLength * sign;
			normalY = distanceY * inverseLength * sign;
		}
	} else {
		const inverseDistance = 1 / distance;
		normalX = (circleX - contactX) * inverseDistance * sign;
		normalY = (circleY - contactY) * inverseDistance * sign;
	}
	return {
		penetration: penetration,
		normalX: normalX,
		normalY: normalY
	};
}

function processEdges(instancePoints: number[], otherPoints: number[], instanceCenter: Vector2, otherCenter: Vector2): ContactResult | null {
	let minimumPenetrationSquareNormal: number = Infinity;
	let bestormalX: number = 0;
	let bestNormalY: number = 0;
	const betweenX: number = otherCenter.x - instanceCenter.x;
	const betweenY: number = otherCenter.y - instanceCenter.y;
	const instancePointCount: number = instancePoints.length;
	const otherPointCount: number = otherPoints.length;
	for (let i = 0, j = instancePointCount - 2; i < instancePointCount; j = i, i += 2) {
		const currentVectorX: number = instancePoints[i];
		const currentVectorY: number = instancePoints[i + 1];
		const previousVectorX: number = instancePoints[j];
		const previousVectorY: number = instancePoints[j + 1];
		const edgeX: number = currentVectorX - previousVectorX;
		const edgeY: number = currentVectorY - previousVectorY;
		const axisUnnormalizedX: number = edgeY;
		const axisUnnormalizedY: number = -edgeX;
		const lengthSquared: number = axisUnnormalizedX * axisUnnormalizedX + axisUnnormalizedY * axisUnnormalizedY;
		if (lengthSquared < 1e-18) {
			continue;
		}
		let instanceMinimumUnnormalized: number = instancePoints[0] * axisUnnormalizedX + instancePoints[1] * axisUnnormalizedY;
		let instanceMaximumUnnormalized: number = instanceMinimumUnnormalized;
		for (let k: number = 2; k < instancePointCount; k += 2) {
			const projection: number = instancePoints[k] * axisUnnormalizedX + instancePoints[k + 1] * axisUnnormalizedY;
			if (projection < instanceMinimumUnnormalized) {
				instanceMinimumUnnormalized = projection;
			} else if (projection > instanceMaximumUnnormalized) {
				instanceMaximumUnnormalized = projection;
			}
		}
		let otherMinimumUnnormalized: number = otherPoints[0] * axisUnnormalizedX + otherPoints[1] * axisUnnormalizedY;
		let otherMaximumUnnormalized: number = otherMinimumUnnormalized;
		for (let k: number = 2; k < otherPointCount; k += 2) {
			const projection: number = otherPoints[k] * axisUnnormalizedX + otherPoints[k + 1] * axisUnnormalizedY;
			if (projection < otherMinimumUnnormalized) {
				otherMinimumUnnormalized = projection;
			} else if (projection > otherMaximumUnnormalized) {
				otherMaximumUnnormalized = projection;
			}
		}
		const overlapUnnorm: number = (instanceMaximumUnnormalized < otherMaximumUnnormalized ? instanceMaximumUnnormalized : otherMaximumUnnormalized) - (instanceMinimumUnnormalized > otherMinimumUnnormalized ? instanceMinimumUnnormalized : otherMinimumUnnormalized);
		if (overlapUnnorm < 0) {
			return null;
		}
		const penetrationSquareNormal: number = (overlapUnnorm * overlapUnnorm) / lengthSquared;
		if (penetrationSquareNormal < minimumPenetrationSquareNormal) {
			minimumPenetrationSquareNormal = penetrationSquareNormal;
			const inverseLength: number = 1 / Math.sqrt(lengthSquared);
			const normalX: number = axisUnnormalizedX * inverseLength;
			const normalY: number = axisUnnormalizedY * inverseLength;
			const dot: number = betweenX * normalX + betweenY * normalY;
			if (dot < 0) {
				bestormalX = -normalX;
				bestNormalY = -normalY;
			} else {
				bestormalX = normalX;
				bestNormalY = normalY;
			}
		}
	}
	if (minimumPenetrationSquareNormal === Infinity) {
		return {
			penetration: Infinity,
			normalX: 0,
			normalY: 0
		};
	}
	return {
		penetration: Math.sqrt(minimumPenetrationSquareNormal),
		normalX: bestormalX,
		normalY: bestNormalY
	};
}

export function computeSATConvex(instancePoints: number[], otherPoints: number[]): SATResult {
	const instanceCenter: Vector2 = computeAveragePoint(instancePoints);
	const otherCenter: Vector2 = computeAveragePoint(otherPoints);
	const instanceContactResult: ContactResult | null = processEdges(instancePoints, otherPoints, instanceCenter, otherCenter);
	if (instanceContactResult === null) {
		return {
			penetration: 0,
			normalX: 0,
			normalY: 0,
			collided: false
		};
	}
	const otherContactResult: ContactResult | null = processEdges(otherPoints, instancePoints, otherCenter, instanceCenter);
	if (otherContactResult === null) {
		return {
			penetration: 0,
			normalX: 0,
			normalY: 0,
			collided: false
		};
	}
	if (instanceContactResult.penetration < otherContactResult.penetration) {
		return {
			penetration: instanceContactResult.penetration,
			normalX: instanceContactResult.normalX,
			normalY: instanceContactResult.normalY,
			collided: true
		};
	} else {
		return {
			penetration: otherContactResult.penetration,
			normalX: -otherContactResult.normalX,
			normalY: -otherContactResult.normalY,
			collided: true
		};
	}
}

export function projectOntoEdge(px: number, py: number, ex0: number, ey0: number, ex1: number, ey1: number, best: BestProjectionState): void {
	const distanceX: number = ex1 - ex0;
	const distanceY: number = ey1 - ey0;
	const lengthSquared: number = distanceX * distanceX + distanceY * distanceY;
	let projectionX: number;
	let projectionY: number;
	if (lengthSquared === 0) {
		projectionX = ex0;
		projectionY = ey0;
	} else {
		const t: number = ((px - ex0) * distanceX + (py - ey0) * distanceY) / lengthSquared;
		if (t <= 0) {
			projectionX = ex0;
			projectionY = ey0;
		} else if (t >= 1) {
			projectionX = ex1;
			projectionY = ey1;
		} else {
			projectionX = ex0 + t * distanceX;
			projectionY = ey0 + t * distanceY;
		}
	}
	const differenceX: number = px - projectionX;
	const differenceY: number = py - projectionY;
	const distanceSquared: number = differenceX * differenceX + differenceY * differenceY;
	if (distanceSquared < best.distanceSquared) {
		best.distanceSquared = distanceSquared;
		best.x = differenceX;
		best.y = differenceY;
	}
}

export function findContactPoint(instanceTriangle: Triangle, otherTriangle: Triangle): Vector2 {
	let sumX: number = 0;
	let sumY: number = 0;
	let count: number = 0;
	for (let i = 0; i < 6; i += 2) {
		const px: number = instanceTriangle[i];
		const py: number = instanceTriangle[i + 1];
		if (isPointInTriangle(px, py, otherTriangle)) {
			sumX += px;
			sumY += py;
			count++;
		}
	}
	for (let i = 0; i < 6; i += 2) {
		const px: number = otherTriangle[i];
		const py: number = otherTriangle[i + 1];
		if (isPointInTriangle(px, py, instanceTriangle)) {
			sumX += px;
			sumY += py;
			count++;
		}
	}
	if (count > 0) {
		const invCount: number = 1 / count;
		return {
			x: sumX * invCount,
			y: sumY * invCount,
		};
	}
	const best: BestProjectionState = {
		distanceSquared: Infinity,
		x: 0,
		y: 0,
	};
	const i0x: number = instanceTriangle[0];
	const i0y: number = instanceTriangle[1];
	const i1x: number = instanceTriangle[2];
	const i1y: number = instanceTriangle[3];
	const i2x: number = instanceTriangle[4];
	const i2y: number = instanceTriangle[5];
	const o0x: number = otherTriangle[0];
	const o0y: number = otherTriangle[1];
	const o1x: number = otherTriangle[2];
	const o1y: number = otherTriangle[3];
	const o2x: number = otherTriangle[4];
	const o2y: number = otherTriangle[5];
	projectOntoEdge(i0x, i0y, o0x, o0y, o1x, o1y, best);
	projectOntoEdge(i0x, i0y, o1x, o1y, o2x, o2y, best);
	projectOntoEdge(i0x, i0y, o2x, o2y, o0x, o0y, best);
	projectOntoEdge(i1x, i1y, o0x, o0y, o1x, o1y, best);
	projectOntoEdge(i1x, i1y, o1x, o1y, o2x, o2y, best);
	projectOntoEdge(i1x, i1y, o2x, o2y, o0x, o0y, best);
	projectOntoEdge(i2x, i2y, o0x, o0y, o1x, o1y, best);
	projectOntoEdge(i2x, i2y, o1x, o1y, o2x, o2y, best);
	projectOntoEdge(i2x, i2y, o2x, o2y, o0x, o0y, best);
	projectOntoEdge(o0x, o0y, i0x, i0y, i1x, i1y, best);
	projectOntoEdge(o0x, o0y, i1x, i1y, i2x, i2y, best);
	projectOntoEdge(o0x, o0y, i2x, i2y, i0x, i0y, best);
	projectOntoEdge(o1x, o1y, i0x, i0y, i1x, i1y, best);
	projectOntoEdge(o1x, o1y, i1x, i1y, i2x, i2y, best);
	projectOntoEdge(o1x, o1y, i2x, i2y, i0x, i0y, best);
	projectOntoEdge(o2x, o2y, i0x, i0y, i1x, i1y, best);
	projectOntoEdge(o2x, o2y, i1x, i1y, i2x, i2y, best);
	projectOntoEdge(o2x, o2y, i2x, i2y, i0x, i0y, best);
	return {
		x: best.x,
		y: best.y,
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

export interface Projection {
	x: number;
	y: number;
	distanceSquared: number;
}

export interface ProjectionRange {
	minimum: number;
	maximum: number;
}

export interface BestProjectionState {
	x: number;
	y: number;
	distanceSquared: number;
}

export function projectPointOnEdge(pointX: number, pointY: number, vertex1X: number, vertex1Y: number, vertex2X: number, vertex2Y: number): Projection {
	const differenceX: number = pointX - vertex1X;
	const differenceY: number = pointY - vertex1Y;
	const edgeVertexX: number = vertex2X - vertex1X;
	const edgeVertexY: number = vertex2Y - vertex1Y;
	const edgeLengthSquared: number = edgeVertexX * edgeVertexX + edgeVertexY * edgeVertexY;
	if (edgeLengthSquared < 1e-9) {
		return {
			x: vertex1X,
			y: vertex1Y,
			distanceSquared: differenceX * differenceX + differenceY * differenceY,
		};
	}
	const dotProduct: number = differenceX * edgeVertexX + differenceY * edgeVertexY;
	if (dotProduct < 0) {
		return {
			x: vertex1X,
			y: vertex1Y,
			distanceSquared: differenceX * differenceX + differenceY * differenceY,
		};
	}
	if (dotProduct > edgeLengthSquared) {
		const differenceX2: number = pointX - vertex2X;
		const differenceY2: number = pointY - vertex2Y;
		return {
			x: vertex2X,
			y: vertex2Y,
			distanceSquared: differenceX2 * differenceX2 + differenceY2 * differenceY2,
		};
	}
	const projectionFactor: number = dotProduct / edgeLengthSquared;
	const projectionX: number = vertex1X + projectionFactor * edgeVertexX;
	const projectionY: number = vertex1Y + projectionFactor * edgeVertexY;
	const distanceToProjectionX: number = pointX - projectionX;
	const distanceToProjectionY: number = pointY - projectionY;
	return {
		x: projectionX,
		y: projectionY,
		distanceSquared: distanceToProjectionX * distanceToProjectionX + distanceToProjectionY * distanceToProjectionY,
	};
}

export function projectPolygon(axisX: number, axisY: number, points: number[]): ProjectionRange {
	let projection: number = points[0] * axisX + points[1] * axisY;
	let minimum: number = projection;
	let maximum: number = projection;
	for (let i: number = 2; i < points.length; i += 2) {
		projection = points[i] * axisX + points[i + 1] * axisY;
		if (projection < minimum) {
			minimum = projection;
		} else if (projection > maximum) {
			maximum = projection;
		}
	}
	return {
		minimum: minimum,
		maximum: maximum
	};
}

export type Triangle = [number, number, number, number, number, number];

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