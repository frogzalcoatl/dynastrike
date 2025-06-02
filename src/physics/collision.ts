import { Entity } from "../entity/entity";
import { Vector2 } from "../geometry/vector";

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
			const edgeX: number = vertex2X - vertex1X;
			const edgeY: number = vertex2Y - vertex1Y;
			const edgeLengthSquared = edgeX * edgeX + edgeY * edgeY;
			if (edgeLengthSquared < 1e-12) {
				continue;
			}
			let projectionFactor = ((pointX - vertex1X) * edgeX + (pointY - vertex1Y) * edgeY) / edgeLengthSquared;
			if (projectionFactor < 0) {
				projectionFactor = 0;
			} else if (projectionFactor > 1) {
				projectionFactor = 1;
			}
			const projectionX: number = vertex1X + projectionFactor * edgeX;
			const projectionY: number = vertex1Y + projectionFactor * edgeY;
			const deltaX: number = pointX - projectionX;
			const deltaY: number = pointY - projectionY;
			const projectionDistanceSquared: number = deltaX * deltaX + deltaY * deltaY;
			if (projectionDistanceSquared < minimumDistanceSquared) {
				minimumDistanceSquared = projectionDistanceSquared;
				closestX = projectionX;
				closestY = projectionY;
			}
		}
		return {
			x: closestX,
			y: closestY
		};
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		if (polygonEntity.points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleRadiusSquared: number = circleEntity.radius * circleEntity.radius;
		const isInside: boolean = this.isPointInPolygon(circleEntity.position.x, circleEntity.position.y, polygonEntity.points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedProjectedXTimesPenetration: number = 0;
		let accumulatedProjectedYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let determinedNormalX: number = 0;
		let determinedNormalY: number = 0;
		let currentPenetration: number = 0;
		let previousPointX: number = polygonEntity.points[polygonEntity.points.length - 2];
		let previousPointY: number = polygonEntity.points[polygonEntity.points.length - 1];
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const currentPointX: number = polygonEntity.points[i];
			const currentPointY: number = polygonEntity.points[i + 1];
			const edgeVectorX: number = currentPointX - previousPointX;
			const edgeVectorY: number = currentPointY - previousPointY;
			const edgeLengthSquared: number = edgeVectorX * edgeVectorX + edgeVectorY * edgeVectorY;
			if (edgeLengthSquared < 1e-12) {
				previousPointX = currentPointX;
				previousPointY = currentPointY;
				continue;
			}
			const dotProductCircleCenterToPreviousPointAndEdge: number = ((circleEntity.position.x - previousPointX) * edgeVectorX + (circleEntity.position.y - previousPointY) * edgeVectorY);
			let projectionFactor: number = dotProductCircleCenterToPreviousPointAndEdge / edgeLengthSquared;
			if (projectionFactor < 0) {
				projectionFactor = 0;
			} else if (projectionFactor > 1) {
				projectionFactor = 1;
			}
			const projectedPointX: number = previousPointX + projectionFactor * edgeVectorX;
			const projectedPointY: number = previousPointY + projectionFactor * edgeVectorY;
			const deltaCircleToProjectionX: number = circleEntity.position.x - projectedPointX;
			const deltaCircleToProjectionY: number = circleEntity.position.y - projectedPointY;
			const distanceToProjectionSquared: number = deltaCircleToProjectionX * deltaCircleToProjectionX + deltaCircleToProjectionY * deltaCircleToProjectionY;
			if (!isInside) {
				if (distanceToProjectionSquared < circleRadiusSquared) {
					const distanceToProjection: number = Math.sqrt(distanceToProjectionSquared);
					currentPenetration = circleEntity.radius - distanceToProjection;
					if (distanceToProjection < 1e-9) {
						const fallbackDeltaX: number = circleEntity.position.x - polygonEntity.position.x;
						const fallbackDeltaY: number = circleEntity.position.y - polygonEntity.position.y;
						const fallbackLength: number = Math.sqrt(fallbackDeltaX * fallbackDeltaX + fallbackDeltaY * fallbackDeltaY);
						if (fallbackLength < 1e-9) {
							determinedNormalX = 0; determinedNormalY = 1;
						} else {
							const inverseFallbackLength: number = 1 / fallbackLength;
							determinedNormalX = fallbackDeltaX * inverseFallbackLength;
							determinedNormalY = fallbackDeltaY * inverseFallbackLength;
						}
					} else {
						const inverseDistanceToProjection: number = 1 / distanceToProjection;
						determinedNormalX = deltaCircleToProjectionX * inverseDistanceToProjection;
						determinedNormalY = deltaCircleToProjectionY * inverseDistanceToProjection;
					}
					sumPenetration += currentPenetration;
					accumulatedNormalXTimesPenetration += determinedNormalX * currentPenetration;
					accumulatedNormalYTimesPenetration += determinedNormalY * currentPenetration;
					accumulatedProjectedXTimesPenetration += projectedPointX * currentPenetration;
					accumulatedProjectedYTimesPenetration += projectedPointY * currentPenetration;
					contactCount++;
				}
			} else {
				const distanceToProjection: number = Math.sqrt(distanceToProjectionSquared);
				currentPenetration = circleEntity.radius + distanceToProjection;
				if (distanceToProjection < 1e-9) {
					const fallbackDeltaX: number = circleEntity.position.x - polygonEntity.position.x;
					const fallbackDeltaY: number = circleEntity.position.y - polygonEntity.position.y;
					const fallbackLength: number = Math.sqrt(fallbackDeltaX * fallbackDeltaX + fallbackDeltaY * fallbackDeltaY);
					if (fallbackLength < 1e-9) {
						determinedNormalX = 0; determinedNormalY = -1;
					} else {
						const inverseFallbackLength: number = 1 / fallbackLength;
						determinedNormalX = -fallbackDeltaX * inverseFallbackLength;
						determinedNormalY = -fallbackDeltaY * inverseFallbackLength;
					}
				} else {
					const inverseDistanceToProjection: number = 1 / distanceToProjection;
					determinedNormalX = -deltaCircleToProjectionX * inverseDistanceToProjection;
					determinedNormalY = -deltaCircleToProjectionY * inverseDistanceToProjection;
				}
				sumPenetration += currentPenetration;
				accumulatedNormalXTimesPenetration += determinedNormalX * currentPenetration;
				accumulatedNormalYTimesPenetration += determinedNormalY * currentPenetration;
				accumulatedProjectedXTimesPenetration += projectedPointX * currentPenetration;
				accumulatedProjectedYTimesPenetration += projectedPointY * currentPenetration;
				contactCount++;
			}
			previousPointX = currentPointX;
			previousPointY = currentPointY;
		}
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const vertexX: number = polygonEntity.points[i];
			const vertexY: number = polygonEntity.points[i + 1];
			const deltaCircleToVertexX: number = circleEntity.position.x - vertexX;
			const deltaCircleToVertexY: number = circleEntity.position.y - vertexY;
			const distanceToVertexSquared: number = deltaCircleToVertexX * deltaCircleToVertexX + deltaCircleToVertexY * deltaCircleToVertexY;
			if (!isInside) {
				if (distanceToVertexSquared < circleRadiusSquared) {
					const distanceToVertex: number = Math.sqrt(distanceToVertexSquared);
					currentPenetration = circleEntity.radius - distanceToVertex;
					if (distanceToVertex < 1e-9) {
						const fallbackDeltaX: number = circleEntity.position.x - polygonEntity.position.x;
						const fallbackDeltaY: number = circleEntity.position.y - polygonEntity.position.y;
						const fallbackLength: number = Math.sqrt(fallbackDeltaX * fallbackDeltaX + fallbackDeltaY * fallbackDeltaY);
						if (fallbackLength < 1e-9) {
							determinedNormalX = 0; determinedNormalY = 1;
						} else {
							const inverseFallbackLength: number = 1 / fallbackLength;
							determinedNormalX = fallbackDeltaX * inverseFallbackLength;
							determinedNormalY = fallbackDeltaY * inverseFallbackLength;
						}
					} else {
						const inverseDistanceToVertex: number = 1 / distanceToVertex;
						determinedNormalX = deltaCircleToVertexX * inverseDistanceToVertex;
						determinedNormalY = deltaCircleToVertexY * inverseDistanceToVertex;
					}
					sumPenetration += currentPenetration;
					accumulatedNormalXTimesPenetration += determinedNormalX * currentPenetration;
					accumulatedNormalYTimesPenetration += determinedNormalY * currentPenetration;
					accumulatedProjectedXTimesPenetration += vertexX * currentPenetration;
					accumulatedProjectedYTimesPenetration += vertexY * currentPenetration;
					contactCount++;
				}
			} else {
				const distanceToVertex: number = Math.sqrt(distanceToVertexSquared);
				currentPenetration = circleEntity.radius + distanceToVertex;
				if (distanceToVertex < 1e-9) {
					const fallbackDeltaX: number = circleEntity.position.x - polygonEntity.position.x;
					const fallbackDeltaY: number = circleEntity.position.y - polygonEntity.position.y;
					const fallbackLength: number = Math.sqrt(fallbackDeltaX * fallbackDeltaX + fallbackDeltaY * fallbackDeltaY);
					if (fallbackLength < 1e-9) {
						determinedNormalX = 0; determinedNormalY = -1;
					} else {
						const inverseFallbackLength: number = 1 / fallbackLength;
						determinedNormalX = -fallbackDeltaX * inverseFallbackLength;
						determinedNormalY = -fallbackDeltaY * inverseFallbackLength;
					}
				} else {
					const inverseDistanceToVertex: number = 1 / distanceToVertex;
					determinedNormalX = -deltaCircleToVertexX * inverseDistanceToVertex;
					determinedNormalY = -deltaCircleToVertexY * inverseDistanceToVertex;
				}
				sumPenetration += currentPenetration;
				accumulatedNormalXTimesPenetration += determinedNormalX * currentPenetration;
				accumulatedNormalYTimesPenetration += determinedNormalY * currentPenetration;
				accumulatedProjectedXTimesPenetration += vertexX * currentPenetration;
				accumulatedProjectedYTimesPenetration += vertexY * currentPenetration;
				contactCount++;
			}
		}
		if (contactCount === 0) {
			return;
		}
		let weightedAverageNormalX: number = 0;
		let weightedAverageNormalY: number = 0;
		let averageContactPointX: number = 0;
		let averageContactPointY: number = 0;
		if (sumPenetration < 1e-9) {
			weightedAverageNormalX = 0.0;
			weightedAverageNormalY = 0.0;
			averageContactPointX = 0.0;
			averageContactPointY = 0.0;
		} else {
			const inverseSumPenetration = 1 / sumPenetration;
			weightedAverageNormalX = accumulatedNormalXTimesPenetration * inverseSumPenetration;
			weightedAverageNormalY = accumulatedNormalYTimesPenetration * inverseSumPenetration;
			averageContactPointX = accumulatedProjectedXTimesPenetration * inverseSumPenetration;
			averageContactPointY = accumulatedProjectedYTimesPenetration * inverseSumPenetration;
		}
		const averageNormalLength = Math.sqrt(weightedAverageNormalX * weightedAverageNormalX + weightedAverageNormalY * weightedAverageNormalY);
		let finalAverageNormalX: number;
		let finalAverageNormalY: number;
		if (averageNormalLength < 1e-9) {
			finalAverageNormalX = 0;
			finalAverageNormalY = 1;
		} else {
			const inverseAverageNormalLength: number = 1 / averageNormalLength;
			finalAverageNormalX = weightedAverageNormalX * inverseAverageNormalLength;
			finalAverageNormalY = weightedAverageNormalY * inverseAverageNormalLength;
		}
		const averagePenetration: number = sumPenetration / contactCount;
		const correctionMagnitude: number = averagePenetration;
		if (polygonEntity.isStatic) {
			circleEntity.moveBy(finalAverageNormalX * correctionMagnitude, finalAverageNormalY * correctionMagnitude);
		} else if (circleEntity.isStatic) {
			polygonEntity.moveBy(-finalAverageNormalX * correctionMagnitude, -finalAverageNormalY * correctionMagnitude);
		} else {
			const totalMass = polygonEntity.mass + circleEntity.mass;
			let inverseTotalMass = 0;
			if (totalMass > 1e-9) {
				inverseTotalMass = 1 / totalMass;
			}
			const polygonFraction: number = circleEntity.mass * inverseTotalMass;
			const circleFraction: number = polygonEntity.mass * inverseTotalMass;
			polygonEntity.moveBy(-finalAverageNormalX * correctionMagnitude * polygonFraction, -finalAverageNormalY * correctionMagnitude * polygonFraction);
			circleEntity.moveBy(finalAverageNormalX * correctionMagnitude * circleFraction, finalAverageNormalY * correctionMagnitude * circleFraction);
		}
		const relativePolygonContactX: number = averageContactPointX - polygonEntity.position.x;
		const relativePolygonContactY: number = averageContactPointY - polygonEntity.position.y;
		const relativeCircleContactX: number = averageContactPointX - circleEntity.position.x;
		const relativeCircleContactY: number = averageContactPointY - circleEntity.position.y;
		const relativeVelocityX: number = (circleEntity.velocity.x - circleEntity.angularVelocity * relativeCircleContactY) - (polygonEntity.velocity.x - polygonEntity.angularVelocity * relativePolygonContactY);
		const relativeVelocityY: number = (circleEntity.velocity.y + circleEntity.angularVelocity * relativeCircleContactX) - (polygonEntity.velocity.y + polygonEntity.angularVelocity * relativePolygonContactX);
		const velocityAlongNormal: number = relativeVelocityX * finalAverageNormalX + relativeVelocityY * finalAverageNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inverseMassPolygon: number = polygonEntity.isStatic ? 0 : 1 / polygonEntity.mass;
		const inverseMassCircle: number = circleEntity.isStatic ? 0 : 1 / circleEntity.mass;
		let inverseInertiaPolygon = 0;
		if (!polygonEntity.isStatic && polygonEntity.inertia > 1e-9) {
			inverseInertiaPolygon = 1 / polygonEntity.inertia;
		}
		let inverseInertiaCircle = 0;
		if (!circleEntity.isStatic && circleEntity.inertia > 1e-9) {
			inverseInertiaCircle = 1 / circleEntity.inertia;
		}
		const perpendicularRelativePolygonContactDotNormal = relativePolygonContactX * finalAverageNormalY - relativePolygonContactY * finalAverageNormalX;
		const perpendicularRelativeCircleContactDotNormal = relativeCircleContactX * finalAverageNormalY - relativeCircleContactY * finalAverageNormalX;
		const denominator = inverseMassPolygon + inverseMassCircle + (perpendicularRelativePolygonContactDotNormal * perpendicularRelativePolygonContactDotNormal) * inverseInertiaPolygon + (perpendicularRelativeCircleContactDotNormal * perpendicularRelativeCircleContactDotNormal) * inverseInertiaCircle;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = -2.0 * velocityAlongNormal / denominator;
		const impulseVectorX: number = impulseMagnitude * finalAverageNormalX;
		const impulseVectorY: number = impulseMagnitude * finalAverageNormalY;
		if (!polygonEntity.isStatic) {
			polygonEntity.velocity.x -= impulseVectorX * inverseMassPolygon;
			polygonEntity.velocity.y -= impulseVectorY * inverseMassPolygon;
			polygonEntity.angularVelocity -= (relativePolygonContactX * impulseVectorY - relativePolygonContactY * impulseVectorX) * inverseInertiaPolygon;
		}
		if (!circleEntity.isStatic) {
			circleEntity.velocity.x += impulseVectorX * inverseMassCircle;
			circleEntity.velocity.y += impulseVectorY * inverseMassCircle;
			circleEntity.angularVelocity += (relativeCircleContactX * impulseVectorY - relativeCircleContactY * impulseVectorX) * inverseInertiaCircle;
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
			this.collidePolygonCircle(instance, other);
			return;
		}
		if (instance.points === null && other.points !== null) {
			this.collidePolygonCircle(other, instance);
			return;
		}
		if (instance.points !== null && other.points !== null) {
			this.collidePolygonPolygon(instance, other);
			return;
		}
	}
}