export interface ProjectionRange {
	minimum: number;
	maximum: number;
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