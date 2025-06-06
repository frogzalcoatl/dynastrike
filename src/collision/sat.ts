import { computeAveragePoint } from "../geometry/polygon";
import { ContactResult, SATResult, Vector2 } from "../types";

export function processEdges(instancePoints: number[], otherPoints: number[], instanceCenter: Vector2, otherCenter: Vector2): ContactResult | null {
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