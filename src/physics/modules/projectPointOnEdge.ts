export interface Projection {
	x: number;
	y: number;
	distanceSquared: number;
}

export function projectPointOnEdge(pointX: number, pointY: number, vertex1X: number, vertex1Y: number, vertex2X: number, vertex2Y: number): Projection {
	const edgeX: number = vertex2X - vertex1X;
	const edgeY: number = vertex2Y - vertex1Y;
	const edgeLengthSquared: number = edgeX * edgeX + edgeY * edgeY;
	if (edgeLengthSquared < 1e-12) {
		const distanceX = pointX - vertex1X;
		const distanceY = pointY - vertex1Y;
		return {
			x: vertex1X,
			y: vertex1Y,
			distanceSquared: distanceX * distanceX + distanceY * distanceY,
		};
	}
	let projectionFactor: number = ((pointX - vertex1X) * edgeX + (pointY - vertex1Y) * edgeY) / edgeLengthSquared;
	if (projectionFactor < 0) {
		projectionFactor = 0;
	} else if (projectionFactor > 1) {
		projectionFactor = 1;
	}
	const projectionX: number = vertex1X + projectionFactor * edgeX;
	const projectionY: number = vertex1Y + projectionFactor * edgeY;
	const distanceX: number = pointX - projectionX;
	const distanceY: number = pointY - projectionY;
	return {
		x: projectionX,
		y: projectionY,
		distanceSquared: distanceX * distanceX + distanceY * distanceY,
	};
}