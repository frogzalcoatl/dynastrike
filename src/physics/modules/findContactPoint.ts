import { Vector2 } from "../../geometry/vector";
import { isPointInTriangle, Triangle } from "./triangle";

interface ProjectionResult {
	x: number;
	y: number;
	distanceSquared: number;
}

function projectPointOntoEdge(pointX: number, pointY: number, edge1X: number, edge1Y: number, edge2X: number, edge2Y: number): ProjectionResult {
	const edgeDistanceX: number = edge2X - edge1X;
	const edgeDistanceY: number = edge2Y - edge1Y;
	const lengthSquared: number = edgeDistanceX * edgeDistanceX + edgeDistanceY * edgeDistanceY;
	let projectionX: number = 0;
	let projectionY: number = 0;
	if (lengthSquared === 0) {
		projectionX = edge1X;
		projectionY = edge1Y;
	} else {
		const projectionFactor: number = ((pointX - edge1X) * edgeDistanceX + (pointY - edge1Y) * edgeDistanceY) / lengthSquared;
		if (projectionFactor <= 0) {
			projectionX = edge1X;
			projectionY = edge1Y;
		} else if (projectionFactor >= 1) {
			projectionX = edge2X;
			projectionY = edge2Y;
		} else {
			projectionX = edge1X + projectionFactor * edgeDistanceX;
			projectionY = edge1Y + projectionFactor * edgeDistanceY;
		}
	}
	const distanceX: number = pointX - projectionX;
	const distanceY: number = pointY - projectionY;
	return {
		x: distanceX,
		y: distanceY,
		distanceSquared: distanceX * distanceX + distanceY * distanceY
	};
}

interface BestProjectionState {
	distanceSquared: number;
	x: number;
	y: number;
}

function updateBestProjection(px: number, py: number, ex0: number, ey0: number, ex1: number, ey1: number, currentBest: BestProjectionState): void {
	const projection: ProjectionResult = projectPointOntoEdge(px, py, ex0, ey0, ex1, ey1);
	if (projection.distanceSquared < currentBest.distanceSquared) {
		currentBest.distanceSquared = projection.distanceSquared;
		currentBest.x = projection.x;
		currentBest.y = projection.y;
	}
}

export function findContactPoint(instanceTriangle: Triangle, otherTriangle: Triangle): Vector2 {
	const contacts: Vector2[] = [];
	for (let i: number = 0; i < 6; i += 2) {
		const pointX: number = instanceTriangle[i];
		const pointY: number = instanceTriangle[i + 1];
		if (isPointInTriangle(pointX, pointY, otherTriangle)) {
			contacts.push({
				x: pointX,
				y: pointY
			});
		}
	}
	for (let i: number = 0; i < 6; i += 2) {
		const pointX: number = otherTriangle[i];
		const pointY: number = otherTriangle[i + 1];
		if (isPointInTriangle(pointX, pointY, instanceTriangle)) {
			contacts.push({
				x: pointX,
				y: pointY
			});
		}
	}
	if (contacts.length > 0) {
		let sumX = 0;
		let sumY = 0;
		const count = contacts.length;
		for (let k: number = 0; k < count; k++) {
			sumX += contacts[k].x;
			sumY += contacts[k].y;
		}
		const inverse: number = 1 / count;
		return {
			x: sumX * inverse,
			y: sumY * inverse
		};
	}
	const bestProjection: BestProjectionState = {
		distanceSquared: Infinity,
		x: 0,
		y: 0
	};
	const instance0X: number = instanceTriangle[0];
	const instance0Y: number = instanceTriangle[1];
	const instance1X: number = instanceTriangle[2];
	const instance1Y: number = instanceTriangle[3];
	const instance2X: number = instanceTriangle[4];
	const instance2Y: number = instanceTriangle[5];
	const other0X: number = otherTriangle[0];
	const other0Y: number = otherTriangle[1];
	const other1X: number = otherTriangle[2];
	const other1Y: number = otherTriangle[3];
	const other2X: number = otherTriangle[4];
	const other2Y: number = otherTriangle[5];
	updateBestProjection(instance0X, instance0Y, other0X, other0Y, other1X, other1Y, bestProjection);
	updateBestProjection(instance0X, instance0Y, other1X, other1Y, other2X, other2Y, bestProjection);
	updateBestProjection(instance0X, instance0Y, other2X, other2Y, other0X, other0Y, bestProjection);
	updateBestProjection(instance1X, instance1Y, other0X, other0Y, other1X, other1Y, bestProjection);
	updateBestProjection(instance1X, instance1Y, other1X, other1Y, other2X, other2Y, bestProjection);
	updateBestProjection(instance1X, instance1Y, other2X, other2Y, other0X, other0Y, bestProjection);
	updateBestProjection(instance2X, instance2Y, other0X, other0Y, other1X, other1Y, bestProjection);
	updateBestProjection(instance2X, instance2Y, other1X, other1Y, other2X, other2Y, bestProjection);
	updateBestProjection(instance2X, instance2Y, other2X, other2Y, other0X, other0Y, bestProjection);
	updateBestProjection(other0X, other0Y, instance0X, instance0Y, instance1X, instance1Y, bestProjection);
	updateBestProjection(other0X, other0Y, instance1X, instance1Y, instance2X, instance2Y, bestProjection);
	updateBestProjection(other0X, other0Y, instance2X, instance2Y, instance0X, instance0Y, bestProjection);
	updateBestProjection(other1X, other1Y, instance0X, instance0Y, instance1X, instance1Y, bestProjection);
	updateBestProjection(other1X, other1Y, instance1X, instance1Y, instance2X, instance2Y, bestProjection);
	updateBestProjection(other1X, other1Y, instance2X, instance2Y, instance0X, instance0Y, bestProjection);
	updateBestProjection(other2X, other2Y, instance0X, instance0Y, instance1X, instance1Y, bestProjection);
	updateBestProjection(other2X, other2Y, instance1X, instance1Y, instance2X, instance2Y, bestProjection);
	updateBestProjection(other2X, other2Y, instance2X, instance2Y, instance0X, instance0Y, bestProjection);
	return {
		x: bestProjection.x,
		y: bestProjection.y
	};
}
