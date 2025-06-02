import { Vector2 } from "../../geometry/vector";
import { computeAveragePoint } from "./computeAveragePoint";
import { ContactResult } from "./computePenetrationAndNormal";

export interface SATResult {
	penetration: number;
	normalX: number;
	normalY: number;
	collided: boolean;
}

function processEdges(instancePoints: number[], otherPoints: number[], instanceCenter: Vector2, otherCenter: Vector2): ContactResult | null {
	let bestPenetration: number = Infinity;
	let bestNormalX: number = 0;
	let bestNormalY: number = 0;
	const betweenX: number = otherCenter.x - instanceCenter.x;
	const betweenY: number = otherCenter.y - instanceCenter.y;
	for (let i: number = 0, j: number = instancePoints.length - 2; i < instancePoints.length; j = i, i += 2) {
		const currentVertexX: number = instancePoints[i];
		const currentVertexY: number = instancePoints[i + 1];
		const previousVertexX = instancePoints[j];
		const previousVertexY = instancePoints[j + 1];
		const edgeX: number = currentVertexX - previousVertexX;
		const edgeY: number = currentVertexY - previousVertexY;
		let axisX: number = edgeY;
		let axisY: number = -edgeX;
		const lengthSquared: number = axisX * axisX + axisY * axisY;
		if (lengthSquared < 1e-18) {
			continue;
		}
		const inverseLength: number = 1 / Math.sqrt(lengthSquared);
		axisX *= inverseLength;
		axisY *= inverseLength;
		let instanceMinimum: number = instancePoints[0] * axisX + instancePoints[1] * axisY;
		let instanceMaximum: number = instanceMinimum;
		for (let k: number = 2; k < instancePoints.length; k += 2) {
			const projection: number = instancePoints[k] * axisX + instancePoints[k + 1] * axisY;
			if (projection < instanceMinimum) {
				instanceMinimum = projection;
			} else if (projection > instanceMaximum) {
				instanceMaximum = projection;
			}
		}
		let otherMinimum: number = otherPoints[0] * axisX + otherPoints[1] * axisY;
		let otherMaximum: number = otherMinimum;
		for (let k: number = 2; k < otherPoints.length; k += 2) {
			const projection = otherPoints[k] * axisX + otherPoints[k + 1] * axisY;
			if (projection < otherMinimum) {
				otherMinimum = projection;
			} else if (projection > otherMaximum) {
				otherMaximum = projection;
			}
		}
		const overlap: number = Math.min(instanceMaximum, otherMaximum) - Math.max(instanceMinimum, otherMinimum);
		if (overlap < 0) {
			return null;
		}
		if (overlap < bestPenetration) {
			bestPenetration = overlap;
			const dot = betweenX * axisX + betweenY * axisY;
			if (dot < 0) {
				bestNormalX = -axisX;
				bestNormalY = -axisY;
			} else {
				bestNormalX = axisX;
				bestNormalY = axisY;
			}
		}
	}
	return {
		penetration: bestPenetration,
		normalX: bestNormalX,
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
			normalX: otherContactResult.normalX,
			normalY: otherContactResult.normalY,
			collided: true
		};
	}
}