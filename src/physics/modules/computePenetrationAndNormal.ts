export interface ContactResult {
	penetration: number;
	normalX: number;
	normalY: number;
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