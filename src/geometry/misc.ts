import { Circle, ContactResult, Projection, ProjectionRange, Triangle, Vector2 } from "../types";
import { isPointInTriangle } from "./polygon";

export function projectOntoEdge(px: number, py: number, ex0: number, ey0: number, ex1: number, ey1: number, best: Projection): void {
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
	const best: Projection = {
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

export function circleFromRadius(boundary: number[], size: number): Circle {
	if (size === 0) {
		return {
			x: 0,
			y: 0,
			radius: 0
		};
	}
	if (size === 2) {
		return {
			x: boundary[0],
			y: boundary[1],
			radius: 0
		};
	}
	if (size === 4) {
		const instancePointX: number = boundary[0];
		const instancePointY: number = boundary[1];
		const otherPointX: number = boundary[2];
		const otherPointY: number = boundary[3];
		const distanceX: number = otherPointX - instancePointX;
		const distanceY: number = otherPointY - instancePointY;
		return {
			x: (instancePointX + otherPointX) * 0.5,
			y: (instancePointY + otherPointY) * 0.5,
			radius: Math.sqrt(distanceX * distanceX + distanceY * distanceY) * 0.5
		};
	}
	const point1X: number = boundary[0];
	const point1Y: number = boundary[1];
	const point2X: number = boundary[2];
	const point2Y: number = boundary[3];
	const point3X: number = boundary[4];
	const point3Y: number = boundary[5];
	const deltaX12: number = point1X - point2X;
	const deltaY12: number = point1Y - point2Y;
	const deltaX23: number = point2X - point3X;
	const deltaY23: number = point2Y - point3Y;
	const deltaX31: number = point3X - point1X;
	const deltaY31: number = point3Y - point1Y;
	const squaredDistance12: number = deltaX12 * deltaX12 + deltaY12 * deltaY12;
	const squaredDistance23: number = deltaX23 * deltaX23 + deltaY23 * deltaY23;
	const squaredDistance31: number = deltaX31 * deltaX31 + deltaY31 * deltaY31;
	if (squaredDistance12 < 1e-12 && squaredDistance23 < 1e-12 && squaredDistance31 < 1e-12) {
		return {
			x: point1X,
			y: point1Y,
			radius: 0
		};
	}
	if (squaredDistance12 < 1e-12 && squaredDistance23 < 1e-12) {
		return {
			x: point1X,
			y: point1Y,
			radius: 0
		};
	}
	if (squaredDistance12 < 1e-12) {
		return {
			x: (point1X + point3X) * 0.5,
			y: (point1Y + point3Y) * 0.5,
			radius: Math.sqrt((point3X - point1X) * (point3X - point1X) + (point3Y - point1Y) * (point3Y - point1Y)) * 0.5
		}
	}
	if (squaredDistance23 < 1e-12) {
		return {
			x: (point2X + point1X) * 0.5,
			y: (point2Y + point1Y) * 0.5,
			radius: Math.sqrt((point1X - point2X) * (point1X - point2X) + (point1Y - point2Y) * (point1Y - point2Y)) * 0.5
		};
	}
	if (squaredDistance31 < 1e-12) {
		return {
			x: (point3X + point2X) * 0.5,
			y: (point3Y + point2Y) * 0.5,
			radius: Math.sqrt((point2X - point3X) * (point2X - point3X) + (point2Y - point3Y) * (point2Y - point3Y)) * 0.5
		};
	}
	let maxSquaredSide: number = squaredDistance12;
	let farPointUX: number = point1X;
	let farPointUY: number = point1Y;
	let farPointVX: number = point2X;
	let farPointVY: number = point2Y;
	if (squaredDistance23 > maxSquaredSide) {
		maxSquaredSide = squaredDistance23;
		farPointUX = point2X;
		farPointUY = point2Y;
		farPointVX = point3X;
		farPointVY = point3Y;
	}
	if (squaredDistance31 > maxSquaredSide) {
		maxSquaredSide = squaredDistance31;
		farPointUX = point3X;
		farPointUY = point3Y;
		farPointVX = point1X;
		farPointVY = point1Y;
	}
	if (maxSquaredSide > squaredDistance12 + squaredDistance23 + squaredDistance31 - maxSquaredSide + 1e-12) {
		return {
			x: (farPointUX + farPointVX) * 0.5,
			y: (farPointUY + farPointVY) * 0.5,
			radius: Math.sqrt(maxSquaredSide) * 0.5
		};
	}
	const determinant: number = 2 * (point1X * (point2Y - point3Y) + point2X * (point3Y - point1Y) + point3X * (point1Y - point2Y));
	if (Math.abs(determinant) < 1e-12) {
		return {
			x: (farPointUX + farPointVX) * 0.5,
			y: (farPointUY + farPointVY) * 0.5,
			radius: Math.sqrt(maxSquaredSide) * 0.5
		};
	}
	const squaredLength1: number = point1X * point1X + point1Y * point1Y;
	const squaredLength2: number = point2X * point2X + point2Y * point2Y;
	const squaredLength3: number = point3X * point3X + point3Y * point3Y;
	const circumcenterX: number = (squaredLength1 * (point2Y - point3Y) + squaredLength2 * (point3Y - point1Y) + squaredLength3 * (point1Y - point2Y)) / determinant;
	const circumcenterY: number = (squaredLength1 * (point3X - point2X) + squaredLength2 * (point1X - point3X) + squaredLength3 * (point2X - point1X)) / determinant;
	const distanceCenterX: number = point1X - circumcenterX;
	const distanceCenterY: number = point1Y - circumcenterY;
	return {
		x: circumcenterX,
		y: circumcenterY,
		radius: Math.sqrt(distanceCenterX * distanceCenterX + distanceCenterY * distanceCenterY)
	};
}
export function welzl(points: number[], pointCount: number, boundary: number[], size: number): Circle {
	if (pointCount === 0 || size === 6) {
		return circleFromRadius(boundary, size);
	}
	const pointIndex: number = pointCount - 1;
	const flatIndex: number = pointIndex << 1;
	const pointX: number = points[flatIndex];
	const pointY: number = points[flatIndex + 1];
	const circleCandidate: Circle = welzl(points, pointIndex, boundary, size);
	const deltaPointX: number = pointX - circleCandidate.x;
	const deltaPointY: number = pointY - circleCandidate.y;
	const squaredDistanceToCenter: number = deltaPointX * deltaPointX + deltaPointY * deltaPointY;
	const squaredRadius: number = circleCandidate.radius * circleCandidate.radius;
	if (circleCandidate.radius > 0 && squaredDistanceToCenter <= squaredRadius + 1e-9) {
		return circleCandidate;
	}
	boundary[size] = pointX;
	boundary[size + 1] = pointY;
	return welzl(points, pointIndex, boundary, size + 2);
}
export function computeMEC(points: number[]): Circle {
	const pointCount: number = points.length >> 1;
	for (let i: number = pointCount - 1; i > 0; --i) {
		const j: number = Math.floor(Math.random() * (i + 1));
		if (j !== i) {
			const ix: number = i << 1;
			const jx: number = j << 1;
			const temporaryX: number = points[ix];
			points[ix] = points[jx];
			points[jx] = temporaryX;
			const temporaryY: number = points[ix + 1];
			points[ix + 1] = points[jx + 1];
			points[jx + 1] = temporaryY;
		}
	}
	return welzl(points, pointCount, [0, 0, 0, 0, 0, 0], 0);
}