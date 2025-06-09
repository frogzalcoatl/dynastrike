import { Circle, Projection } from "../types";

export function projectPointOnEdge(pointX: number, pointY: number, vertex1X: number, vertex1Y: number, vertex2X: number, vertex2Y: number): Projection {
	const differenceX: number = pointX - vertex1X;
	const differenceY: number = pointY - vertex1Y;
	const edgeX: number = vertex2X - vertex1X;
	const edgeY: number = vertex2Y - vertex1Y;
	const edgeLengthSquared: number = edgeX * edgeX + edgeY * edgeY;
	if (edgeLengthSquared < 1e-9) {
		return {
			x: vertex1X,
			y: vertex1Y,
			distanceSquared: differenceX * differenceX + differenceY * differenceY,
		};
	}
	const dotProduct: number = differenceX * edgeX + differenceY * edgeY;
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
	const projectionX: number = vertex1X + projectionFactor * edgeX;
	const projectionY: number = vertex1Y + projectionFactor * edgeY;
	const distanceToProjectionX: number = pointX - projectionX;
	const distanceToProjectionY: number = pointY - projectionY;
	return {
		x: projectionX,
		y: projectionY,
		distanceSquared: distanceToProjectionX * distanceToProjectionX + distanceToProjectionY * distanceToProjectionY,
	};
}

export function circleFromRadius(boundary: number[], size: number, circle: Circle): void {
	if (size === 0) {
		circle.x = 0;
		circle.y = 0;
		circle.radius = 0;
		return;
	}
	if (size === 2) {
		circle.x = boundary[0];
		circle.y = boundary[1];
		circle.radius = 0;
		return;
	}
	if (size === 4) {
		const instancePointX: number = boundary[0];
		const instancePointY: number = boundary[1];
		const otherPointX: number = boundary[2];
		const otherPointY: number = boundary[3];
		const distanceX: number = otherPointX - instancePointX;
		const distanceY: number = otherPointY - instancePointY;
		circle.x = (instancePointX + otherPointX) * 0.5;
		circle.y = (instancePointY + otherPointY) * 0.5;
		circle.radius = Math.sqrt(distanceX * distanceX + distanceY * distanceY) * 0.5;
		return;
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
		circle.x = point1X;
		circle.y = point1Y;
		circle.radius = 0;
		return;
	}
	if (squaredDistance12 < 1e-12 && squaredDistance23 < 1e-12) {
		circle.x = point1X;
		circle.y = point1Y;
		circle.radius = 0;
		return;
	}
	if (squaredDistance12 < 1e-12) {
		circle.x = (point1X + point3X) * 0.5;
		circle.y = (point1Y + point3Y) * 0.5;
		circle.radius = Math.sqrt((point3X - point1X) * (point3X - point1X) + (point3Y - point1Y) * (point3Y - point1Y)) * 0.5;
		return;
	}
	if (squaredDistance23 < 1e-12) {
		circle.x = (point2X + point1X) * 0.5;
		circle.y = (point2Y + point1Y) * 0.5;
		circle.radius = Math.sqrt((point1X - point2X) * (point1X - point2X) + (point1Y - point2Y) * (point1Y - point2Y)) * 0.5;
		return;
	}
	if (squaredDistance31 < 1e-12) {
		circle.x = (point3X + point2X) * 0.5;
		circle.y = (point3Y + point2Y) * 0.5;
		circle.radius = Math.sqrt((point2X - point3X) * (point2X - point3X) + (point2Y - point3Y) * (point2Y - point3Y)) * 0.5;
		return;
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
		circle.x = (farPointUX + farPointVX) * 0.5;
		circle.y = (farPointUY + farPointVY) * 0.5;
		circle.radius = Math.sqrt(maxSquaredSide) * 0.5;
		return;
	}
	const determinant: number = 2 * (point1X * (point2Y - point3Y) + point2X * (point3Y - point1Y) + point3X * (point1Y - point2Y));
	if (Math.abs(determinant) < 1e-12) {
		circle.x = (farPointUX + farPointVX) * 0.5;
		circle.y = (farPointUY + farPointVY) * 0.5;
		circle.radius = Math.sqrt(maxSquaredSide) * 0.5;
		return;
	}
	const squaredLength1: number = point1X * point1X + point1Y * point1Y;
	const squaredLength2: number = point2X * point2X + point2Y * point2Y;
	const squaredLength3: number = point3X * point3X + point3Y * point3Y;
	const circumcenterX: number = (squaredLength1 * (point2Y - point3Y) + squaredLength2 * (point3Y - point1Y) + squaredLength3 * (point1Y - point2Y)) / determinant;
	const circumcenterY: number = (squaredLength1 * (point3X - point2X) + squaredLength2 * (point1X - point3X) + squaredLength3 * (point2X - point1X)) / determinant;
	const distanceCenterX: number = point1X - circumcenterX;
	const distanceCenterY: number = point1Y - circumcenterY;
	circle.x = circumcenterX;
	circle.y = circumcenterY;
	circle.radius = Math.sqrt(distanceCenterX * distanceCenterX + distanceCenterY * distanceCenterY);
}

export function welzl(points: number[], pointCount: number, boundary: number[], size: number, result: Circle = { x: 0, y: 0, radius: 0 }): Circle {
	if (pointCount === 0 || size === 6) {
		circleFromRadius(boundary, size, result);
		return result;
	}
	const pointIndex: number = pointCount - 1;
	const flatIndex: number = pointIndex << 1;
	const pointX: number = points[flatIndex];
	const pointY: number = points[flatIndex + 1];
	const circleCandidate: Circle = welzl(points, pointIndex, boundary, size, result);
	const deltaX: number = pointX - circleCandidate.x;
	const deltaY: number = pointY - circleCandidate.y;
	if (circleCandidate.radius > 0 && (deltaX * deltaX + deltaY * deltaY) <= (circleCandidate.radius * circleCandidate.radius) + 1e-9) {
		return circleCandidate;
	}
	boundary[size] = pointX;
	boundary[size + 1] = pointY;
	return welzl(points, pointIndex, boundary, size + 2, result);
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