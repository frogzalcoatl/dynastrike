export interface Box {
	minX: number;
	minY: number;
	maxX: number;
	maxY: number;
}

export function boxesIntersect(instance: Box, other: Box): boolean {
	return !(instance.minX >= other.maxX || instance.maxX <= other.minX || instance.minY >= other.maxY || instance.maxY <= other.minY);
}

export function computeBox(points: number[]): Box {
	const box = {
		minX: points[0],
		minY: points[1],
		maxX: points[0],
		maxY: points[1]
	};
	for (let i = 2; i < points.length; i += 2) {
		const pointX = points[i];
		const pointY = points[i + 1];
		if (box.minX > pointX) box.minX = pointX;
		if (box.minY > pointY) box.minY = pointY;
		if (box.maxX < pointX) box.maxX = pointX;
		if (box.maxY < pointY) box.maxY = pointY;
	}
	return box;
}
