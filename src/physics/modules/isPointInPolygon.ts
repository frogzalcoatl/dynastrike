export function isPointInPolygon(pointX: number, pointY: number, points: number[]): boolean {
	let isInside: boolean = false;
	let previousPointX: number = points[points.length - 2];
	let previousPointY: number = points[points.length - 1];
	for (let i: number = 0; i < points.length; i += 2) {
		const currentPointX: number = points[i];
		const currentPointY: number = points[i + 1];
		if ((currentPointY > pointY) !== (previousPointY > pointY) && previousPointY - currentPointY !== 0 && pointX < currentPointX + (previousPointX - currentPointX) * (pointY - currentPointY) / (previousPointY - currentPointY)) {
			isInside = !isInside;
		}
		if (pointX === currentPointX && pointY === currentPointY) {
			return true;
		}
		previousPointX = currentPointX;
		previousPointY = currentPointY;
	}
	return isInside;
}