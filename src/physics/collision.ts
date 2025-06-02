import { Entity } from "../entity/entity";
import { Vector2 } from "../geometry/vector";

export interface Projection {
	x: number;
	y: number;
	distanceSquared: number;
}

export interface ContactResult {
	penetration: number;
	normalX: number;
	normalY: number;
}

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance) {
			return;
		}
		if (instance.isStatic && other.isStatic) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		let normalX: number = 0;
		let normalY: number = 0;
		if (distance < 1e-9) {
			const angle: number = Math.PI * 2 * Math.random();
			normalX = Math.cos(angle);
			normalY = Math.sin(angle);
		} else {
			const inverseDistance: number = 1 / distance;
			normalX = distanceX * inverseDistance;
			normalY = distanceY * inverseDistance;
		}
		const overlap: number = minimumDistance - distance;
		if (instance.isStatic) {
			other.moveBy(normalX * overlap, normalY * overlap);
		} else if (other.isStatic) {
			instance.moveBy(-normalX * overlap, -normalY * overlap);
		} else {
			const inverseTotalMass: number = 1 / (instance.mass + other.mass);
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
			other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const velocityAlongNormal = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const otherInverseMass: number = other.isStatic ? 0 : 1 / other.mass;
		const impulseMagnitude = -2 * velocityAlongNormal / (instanceInverseMass + otherInverseMass);
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.velocity.x -= impulseX * instanceInverseMass;
			instance.velocity.y -= impulseY * instanceInverseMass;
		}
		if (!other.isStatic) {
			other.velocity.x += impulseX * otherInverseMass;
			other.velocity.y += impulseY * otherInverseMass;
		}
	}

	private static isPointInPolygon(pointX: number, pointY: number, points: number[]): boolean {
		const length: number = points.length;
		let isInside: boolean = false;
		for (let i: number = 0, j = length - 2; i < length; j = i, i += 2) {
			const currentPointX = points[i];
			const currentPointY = points[i + 1];
			const previousPointX = points[j];
			const previousPointY = points[j + 1];
			if (currentPointY > pointY !== previousPointY > pointY && pointX < (previousPointX - currentPointX) * (pointY - currentPointY) / (previousPointY - currentPointY) + currentPointX) {
				isInside = !isInside;
			}
		}

		return isInside;
	}

	public static getClosestPointOnPolygon(points: number[], pointX: number, pointY: number): Vector2 {
		let closestX: number = 0;
		let closestY: number = 0;
		let minimumDistanceSquared: number = Infinity;
		for (let i: number = 0; i < points.length; i += 2) {
			const vertexX: number = points[i];
			const vertexY: number = points[i + 1];
			const distanceX: number = pointX - vertexX;
			const distanceY: number = pointY - vertexY;
			const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
			if (distanceSquared < minimumDistanceSquared) {
				minimumDistanceSquared = distanceSquared;
				closestX = vertexX;
				closestY = vertexY;
			}
		}
		for (let i: number = 0, j: number = points.length - 2; i < points.length; j = i, i += 2) {
			const vertex1X: number = points[j];
			const vertex1Y: number = points[j + 1];
			const vertex2X: number = points[i];
			const vertex2Y: number = points[i + 1];
			const projection: Projection = this.projectPointOnEdge(pointX, pointY, vertex1X, vertex1Y, vertex2X, vertex2Y);
			if (projection.distanceSquared < minimumDistanceSquared) {
				minimumDistanceSquared = projection.distanceSquared;
				closestX = projection.x;
				closestY = projection.y;
			}
		}
		return {
			x: closestX,
			y: closestY
		};
	}

	private static projectPointOnEdge(pointX: number, pointY: number, vertex1X: number, vertex1Y: number, vertex2X: number, vertex2Y: number): Projection {
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

	private static getPenetrationAndNormal(circleX: number, circleY: number, contactX: number, contactY: number, radius: number, polygonCenterX: number, polygonCenterY: number, isInside: boolean, distanceSquared: number, radiusSquared: number): ContactResult {
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
		if (distance < 1e-9) {
			const distanceX: number = circleX - polygonCenterX;
			const distanceY: number = circleY - polygonCenterY;
			const length: number = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
			if (length < 1e-9) {
				normalX = 0;
				normalY = isInside ? -1 : 1;
			} else {
				const inverseLength: number = 1 / length;
				normalX = distanceX * inverseLength * (isInside ? -1 : 1);
				normalY = distanceY * inverseLength * (isInside ? -1 : 1);
			}
		} else {
			const inverseDistance: number = 1 / distance;
			const sign = isInside ? -1 : 1;
			normalX = (circleX - contactX) * inverseDistance * sign;
			normalY = (circleY - contactY) * inverseDistance * sign;
		}
		return {
			penetration,
			normalX: normalX,
			normalY: normalY
		};
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		if (polygonEntity.points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleRadius: number = circleEntity.radius;
		const circleRadiusSquared: number = circleRadius * circleRadius;
		const isInside: boolean = this.isPointInPolygon(circleEntity.position.x, circleEntity.position.y, polygonEntity.points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedContactXTimesPenetration: number = 0;
		let accumulatedContactYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let previousX: number = polygonEntity.points[polygonEntity.points.length - 2];
		let previousY: number = polygonEntity.points[polygonEntity.points.length - 1];
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const currentX: number = polygonEntity.points[i];
			const currentY: number = polygonEntity.points[i + 1];
			const projection: Projection = this.projectPointOnEdge(circleEntity.position.x, circleEntity.position.y, previousX, previousY, currentX, currentY);
			const contact: ContactResult = this.getPenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, projection.x, projection.y, circleRadius, polygonEntity.position.x, polygonEntity.position.y, isInside, projection.distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += projection.x * contact.penetration;
				accumulatedContactYTimesPenetration += projection.y * contact.penetration;
				contactCount++;
			}
			previousX = currentX;
			previousY = currentY;
		}
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const vertexX: number = polygonEntity.points[i];
			const vertexY: number = polygonEntity.points[i + 1];
			const distanceX: number = circleEntity.position.x - vertexX;
			const distanceY: number = circleEntity.position.y - vertexY;
			const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
			const contact: ContactResult = this.getPenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, vertexX, vertexY, circleRadius, polygonEntity.position.x, polygonEntity.position.y, isInside, distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += vertexX * contact.penetration;
				accumulatedContactYTimesPenetration += vertexY * contact.penetration;
				contactCount++;
			}
		}
		if (contactCount === 0 && isInside) {
			let minimumDistance: number = Infinity;
			let bestNormalX: number = 0;
			let bestNormalY: number = 0;
			let previousX: number = polygonEntity.points[polygonEntity.points.length - 2];
			let previousY: number = polygonEntity.points[polygonEntity.points.length - 1];
			for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
				const currentX: number = polygonEntity.points[i];
				const currentY: number = polygonEntity.points[i + 1];
				const edgeX: number = currentX - previousX;
				const edgeY: number = currentY - previousY;
				const edgeLengthSquared: number = edgeX * edgeX + edgeY * edgeY;
				if (edgeLengthSquared < 1e-12) {
					previousX = currentX;
					previousY = currentY;
					continue;
				}
				let projectionFactor: number = ((circleEntity.position.x - previousX) * edgeX + (circleEntity.position.y - previousY) * edgeY) / edgeLengthSquared;
				if (projectionFactor < 0) {
					projectionFactor = 0;
				} else if (projectionFactor > 1) {
					projectionFactor = 1;
				}
				const projectionX: number = previousX + projectionFactor * edgeX;
				const projectionY: number = previousY + projectionFactor * edgeY;
				const distanceX: number = circleEntity.position.x - projectionX;
				const distanceY: number = circleEntity.position.y - projectionY;
				const distance: number = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
				if (distance < minimumDistance) {
					minimumDistance = distance;
					if (distance < 1e-9) {
						bestNormalX = 0;
						bestNormalY = 1;
					} else {
						const inverseDistance: number = 1 / distance;
						bestNormalX = -distanceX * inverseDistance;
						bestNormalY = -distanceY * inverseDistance;
					}
				}
				previousX = currentX;
				previousY = currentY;
			}
			const penetration: number = circleEntity.radius + minimumDistance;
			if (polygonEntity.isStatic) {
				circleEntity.moveBy(bestNormalX * penetration, bestNormalY * penetration);
			} else if (circleEntity.isStatic) {
				polygonEntity.moveBy(-bestNormalX * penetration, -bestNormalY * penetration);
			} else {
				const totalMass = polygonEntity.mass + circleEntity.mass;
				const invTotal = totalMass > 1e-9 ? 1 / totalMass : 0;
				const polyFrac = circleEntity.mass * invTotal;
				const circFrac = polygonEntity.mass * invTotal;
				polygonEntity.moveBy(-bestNormalX * penetration * polyFrac, -bestNormalY * penetration * polyFrac);
				circleEntity.moveBy(bestNormalX * penetration * circFrac, bestNormalY * penetration * circFrac);
			}
			return;
		}
		if (contactCount === 0) {
			return;
		}
		let weightedNormalX: number = 0;
		let weightedNormalY: number = 0;
		let averageContactX: number = 0;
		let averageContactY: number = 0;
		if (sumPenetration < 1e-9) {
			weightedNormalX = 0;
			weightedNormalY = 0;
			averageContactX = 0;
			averageContactY = 0;
		} else {
			const inverseSum: number = 1 / sumPenetration;
			weightedNormalX = accumulatedNormalXTimesPenetration * inverseSum;
			weightedNormalY = accumulatedNormalYTimesPenetration * inverseSum;
			averageContactX = accumulatedContactXTimesPenetration * inverseSum;
			averageContactY = accumulatedContactYTimesPenetration * inverseSum;
		}
		const avgNormLen: number = Math.sqrt(weightedNormalX * weightedNormalX + weightedNormalY * weightedNormalY);
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		if (avgNormLen < 1e-9) {
			finalNormalX = 0;
			finalNormalY = 1;
		} else {
			const inverseLength: number = 1 / avgNormLen;
			finalNormalX = weightedNormalX * inverseLength;
			finalNormalY = weightedNormalY * inverseLength;
		}
		const averagePenetration: number = sumPenetration / contactCount;
		const correctionMagnitude: number = averagePenetration;
		if (polygonEntity.isStatic) {
			circleEntity.moveBy(finalNormalX * correctionMagnitude, finalNormalY * correctionMagnitude);
		} else if (circleEntity.isStatic) {
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude, -finalNormalY * correctionMagnitude);
		} else {
			const totalMass = polygonEntity.mass + circleEntity.mass;
			const inverseTotalMass = 1 / totalMass;
			const polygonCorrectionFactor = circleEntity.mass * inverseTotalMass;
			const circleCorrectionFactor = polygonEntity.mass * inverseTotalMass;
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude * polygonCorrectionFactor, -finalNormalY * correctionMagnitude * polygonCorrectionFactor);
			circleEntity.moveBy(finalNormalX * correctionMagnitude * circleCorrectionFactor, finalNormalY * correctionMagnitude * circleCorrectionFactor);
		}
		const relativePolygonX: number = averageContactX - polygonEntity.position.x;
		const relativePolygonY: number = averageContactY - polygonEntity.position.y;
		const relativeCircleX: number = averageContactX - circleEntity.position.x;
		const relativeCircleY: number = averageContactY - circleEntity.position.y;
		const relativeVelocityX: number = (circleEntity.velocity.x - circleEntity.angularVelocity * relativeCircleY) - (polygonEntity.velocity.x - polygonEntity.angularVelocity * relativePolygonY);
		const relativeVelocityY: number = (circleEntity.velocity.y + circleEntity.angularVelocity * relativeCircleX) - (polygonEntity.velocity.y + polygonEntity.angularVelocity * relativePolygonX);
		const velocityAlongNormal: number = relativeVelocityX * finalNormalX + relativeVelocityY * finalNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inversePolygonMass: number = polygonEntity.isStatic ? 0 : 1 / polygonEntity.mass;
		const inverseCircleMass: number = circleEntity.isStatic ? 0 : 1 / circleEntity.mass;
		let invInertiaPoly: number = 0;
		if (!polygonEntity.isStatic && polygonEntity.inertia > 1e-9) {
			invInertiaPoly = 1 / polygonEntity.inertia;
		}
		let invInertiaCirc: number = 0;
		if (!circleEntity.isStatic && circleEntity.inertia > 1e-9) {
			invInertiaCirc = 1 / circleEntity.inertia;
		}
		const relativePolygonPerpendicular: number = relativePolygonX * finalNormalY - relativePolygonY * finalNormalX;
		const relativeCirclePerpendicular: number = relativeCircleX * finalNormalY - relativeCircleY * finalNormalX;
		const denominator: number = inversePolygonMass + inverseCircleMass + relativePolygonPerpendicular * relativePolygonPerpendicular * invInertiaPoly + relativeCirclePerpendicular * relativeCirclePerpendicular * invInertiaCirc;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = (-2.0 * velocityAlongNormal) / denominator;
		const impulseX: number = impulseMagnitude * finalNormalX;
		const impulseY: number = impulseMagnitude * finalNormalY;
		if (!polygonEntity.isStatic) {
			polygonEntity.velocity.x -= impulseX * inversePolygonMass;
			polygonEntity.velocity.y -= impulseY * inversePolygonMass;
			polygonEntity.angularVelocity -= (relativePolygonX * impulseY - relativePolygonY * impulseX) * invInertiaPoly;
		}
		if (!circleEntity.isStatic) {
			circleEntity.velocity.x += impulseX * inverseCircleMass;
			circleEntity.velocity.y += impulseY * inverseCircleMass;
			circleEntity.angularVelocity += (relativeCircleX * impulseY - relativeCircleY * impulseX) * invInertiaCirc;
		}
	}

	public static collidePolygonPolygon(instance: Entity, other: Entity): void { // later

	}

	public static collide(instance: Entity, other: Entity): void {
		if (instance.points === null && other.points === null) {
			this.collideCircleCircle(instance, other);
			return;
		}
		if (instance.points !== null && other.points === null) {
			console.time("collidePolygonCircle");
			this.collidePolygonCircle(instance, other);
			console.timeEnd("collidePolygonCircle");
			return;
		}
		if (instance.points === null && other.points !== null) {
			console.time("collidePolygonCircle");
			this.collidePolygonCircle(other, instance);
			console.timeEnd("collidePolygonCircle");
			return;
		}
		if (instance.points !== null && other.points !== null) {
			this.collidePolygonPolygon(instance, other);
			return;
		}
	}
}