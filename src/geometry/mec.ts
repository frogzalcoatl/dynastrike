export interface Circle {
	x: number;
	y: number;
	radius: number;
}

type BoundaryBuffer = number[];

export function circleFromRadius(boundary: BoundaryBuffer, size: number): Circle {
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
			x: (instancePointX + otherPointX) / 2,
			y: (instancePointY + otherPointY) / 2,
			radius: Math.sqrt(distanceX * distanceX + distanceY * distanceY) / 2
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
		const distanceX: number = point3X - point1X;
		const distanceY: number = point3Y - point1Y;
		return {
			x: (point1X + point3X) / 2,
			y: (point1Y + point3Y) / 2,
			radius: Math.sqrt(distanceX * distanceX + distanceY * distanceY) / 2
		}
	}
	if (squaredDistance23 < 1e-12) {
		const distanceX: number = point1X - point2X;
		const distanceY: number = point1Y - point2Y;
		return {
			x: (point2X + point1X) / 2,
			y: (point2Y + point1Y) / 2,
			radius: Math.sqrt(distanceX * distanceX + distanceY * distanceY) / 2
		};
	}
	if (squaredDistance31 < 1e-12) {
		const distanceX: number = point2X - point3X;
		const distanceY: number = point2Y - point3Y;
		return {
			x: (point3X + point2X) / 2,
			y: (point3Y + point2Y) / 2,
			radius: Math.sqrt(distanceX * distanceX + distanceY * distanceY) / 2
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
			x: (farPointUX + farPointVX) / 2,
			y: (farPointUY + farPointVY) / 2,
			radius: Math.sqrt(maxSquaredSide) / 2
		};
	}
	const determinant: number = 2 * (point1X * (point2Y - point3Y) + point2X * (point3Y - point1Y) + point3X * (point1Y - point2Y));
	if (Math.abs(determinant) < 1e-12) {
		return {
			x: (farPointUX + farPointVX) / 2,
			y: (farPointUY + farPointVY) / 2,
			radius: Math.sqrt(maxSquaredSide) / 2
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

export function welzl(points: number[], pointCount: number, boundary: BoundaryBuffer, size: number): Circle {
	if (pointCount === 0 || size === 6) {
		return circleFromRadius(boundary, size);
	}
	const flatIndex: number = (pointCount - 1) << 1;
	const pointX: number = points[flatIndex];
	const pointY: number = points[flatIndex + 1];
	const circleCandidate: Circle = welzl(points, pointCount - 1, boundary, size);
	const deltaPointX: number = pointX - circleCandidate.x;
	const deltaPointY: number = pointY - circleCandidate.y;
	if (circleCandidate.radius > 0 && deltaPointX * deltaPointX + deltaPointY * deltaPointY <= circleCandidate.radius * circleCandidate.radius + 1e-9) {
		return circleCandidate;
	}
	boundary[size] = pointX;
	boundary[size + 1] = pointY;
	const circleWithPoint: Circle = welzl(points, pointCount - 1, boundary, size + 2);
	return circleWithPoint;
}

export function computeMEC(points: number[]): Circle {
	const totalCoordinatesLength: number = points.length;
	const pointCount: number = totalCoordinatesLength >> 1;
	for (let i: number = pointCount - 1; i > 0; --i) {
		const j: number = (Math.random() * (i + 1)) | 0;
		if (j === i) {
			continue;
		}
		const ix: number = i << 1;
		const iy: number = ix + 1;
		const jx: number = j << 1;
		const jy: number = jx + 1;
		const pointX: number = points[ix];
		const pointY: number = points[iy];
		points[ix] = points[jx];
		points[iy] = points[jy];
		points[jx] = pointX;
		points[jy] = pointY;
	}
	return welzl(points, pointCount, [0, 0, 0, 0, 0, 0], 0);
}
