import { Entity } from "../entity/entity";
import { computePenetrationAndNormal, computeSATConvex, ContactResult, findContactPoint, isPointInPolygon, Projection, projectPointOnEdge, Triangle, triangulate } from "./utilities";

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		const distanceX: number = other.positionX - instance.positionX;
		const distanceY: number = other.positionY - instance.positionY;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance || (instance.isStatic && other.isStatic)) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		let normalX: number = 0;
		let normalY: number = 0;
		if (distance < 1e-9) {
			normalX = 0;
			normalY = 1;
		} else {
			const inverseDistance: number = 1 / distance;
			normalX = distanceX * inverseDistance;
			normalY = distanceY * inverseDistance;
		}
		const overlap: number = minimumDistance - distance;
		const overlapCorrectionX: number = normalX * overlap;
		const overlapCorrectionY: number = normalY * overlap;
		if (instance.isStatic) {
			other.moveBy(overlapCorrectionX, overlapCorrectionY);
		} else if (other.isStatic) {
			instance.moveBy(-overlapCorrectionX, -overlapCorrectionY);
		} else {
			const totalMass: number = instance.mass + other.mass;
			const inverseTotalMass: number = 1 / totalMass;
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-overlapCorrectionX * instanceCorrectionFraction, -overlapCorrectionY * instanceCorrectionFraction);
			other.moveBy(overlapCorrectionX * otherCorrectionFraction, overlapCorrectionY * otherCorrectionFraction);
		}
		const relativeVelocityX: number = other.positionalVelocity.x - instance.positionalVelocity.x;
		const relativeVelocityY: number = other.positionalVelocity.y - instance.positionalVelocity.y;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const otherInverseMass: number = other.isStatic ? 0 : 1 / other.mass;
		const sumInverseMass: number = instanceInverseMass + otherInverseMass;
		const impulseMagnitude: number = -2 * velocityAlongNormal / sumInverseMass;
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.positionalVelocity.x -= impulseX * instanceInverseMass;
			instance.positionalVelocity.y -= impulseY * instanceInverseMass;
		}
		if (!other.isStatic) {
			other.positionalVelocity.x += impulseX * otherInverseMass;
			other.positionalVelocity.y += impulseY * otherInverseMass;
		}
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		const points = polygonEntity.points!;
		if (points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleX: number = circleEntity.positionX;
		const circleY: number = circleEntity.positionY;
		const circleRadius: number = circleEntity.radius;
		const circleRadiusSquared: number = circleRadius * circleRadius;
		const isInside: boolean = isPointInPolygon(circleX, circleY, points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedContactXTimesPenetration: number = 0;
		let accumulatedContactYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let minimumDistanceSqForInside: number = Infinity;
		let featureXForInsideNormal: number = 0;
		let featureYForInsideNormal: number = 0;
		const pointCount: number = points.length;
		let previousVertexX: number = points[pointCount - 2];
		let previousVertexY: number = points[pointCount - 1];
		for (let i: number = 0; i < pointCount; i += 2) {
			const currentVertexX: number = points[i];
			const currentVertexY: number = points[i + 1];
			const projection: Projection = projectPointOnEdge(circleX, circleY, previousVertexX, previousVertexY, currentVertexX, currentVertexY);
			const edgeContact: ContactResult = computePenetrationAndNormal(circleX, circleY, projection.x, projection.y, circleRadius, polygonEntity.positionX, polygonEntity.positionY, isInside, projection.distanceSquared, circleRadiusSquared);
			if (edgeContact.penetration > 0) {
				sumPenetration += edgeContact.penetration;
				accumulatedNormalXTimesPenetration += edgeContact.normalX * edgeContact.penetration;
				accumulatedNormalYTimesPenetration += edgeContact.normalY * edgeContact.penetration;
				accumulatedContactXTimesPenetration += projection.x * edgeContact.penetration;
				accumulatedContactYTimesPenetration += projection.y * edgeContact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (projection.distanceSquared < minimumDistanceSqForInside) {
					minimumDistanceSqForInside = projection.distanceSquared;
					featureXForInsideNormal = projection.x;
					featureYForInsideNormal = projection.y;
				}
			}
			const distVertexCircleX: number = circleX - currentVertexX;
			const distVertexCircleY: number = circleY - currentVertexY;
			const vertexDistanceSquared: number = distVertexCircleX * distVertexCircleX + distVertexCircleY * distVertexCircleY;
			const vertexContact: ContactResult = computePenetrationAndNormal(circleX, circleY, currentVertexX, currentVertexY, circleRadius, polygonEntity.positionX, polygonEntity.positionY, isInside, vertexDistanceSquared, circleRadiusSquared);
			if (vertexContact.penetration > 0) {
				sumPenetration += vertexContact.penetration;
				accumulatedNormalXTimesPenetration += vertexContact.normalX * vertexContact.penetration;
				accumulatedNormalYTimesPenetration += vertexContact.normalY * vertexContact.penetration;
				accumulatedContactXTimesPenetration += currentVertexX * vertexContact.penetration;
				accumulatedContactYTimesPenetration += currentVertexY * vertexContact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (vertexDistanceSquared < minimumDistanceSqForInside) {
					minimumDistanceSqForInside = vertexDistanceSquared;
					featureXForInsideNormal = currentVertexX;
					featureYForInsideNormal = currentVertexY;
				}
			}
			previousVertexX = currentVertexX;
			previousVertexY = currentVertexY;
		}
		if (contactCount === 0) {
			if (isInside) {
				if (minimumDistanceSqForInside === Infinity) {
					return;
				}
				const rawNormalToFeatureX: number = featureXForInsideNormal - circleX;
				const rawNormalToFeatureY: number = featureYForInsideNormal - circleY;
				const distanceToBoundary: number = Math.sqrt(minimumDistanceSqForInside);
				let bestNormalX: number = 0;
				let bestNormalY: number = 0;
				if (distanceToBoundary < 1e-9) {
					bestNormalX = 0;
					bestNormalY = 1;
				} else {
					const inverseDistance: number = 1 / distanceToBoundary;
					bestNormalX = rawNormalToFeatureX * inverseDistance;
					bestNormalY = rawNormalToFeatureY * inverseDistance;
				}
				const penetration: number = circleRadius + distanceToBoundary;
				const correctionX: number = bestNormalX * penetration;
				const correctionY: number = bestNormalY * penetration;
				if (polygonEntity.isStatic) {
					circleEntity.moveBy(correctionX, correctionY);
				} else if (circleEntity.isStatic) {
					polygonEntity.moveBy(-correctionX, -correctionY);
				} else {
					const totalMass: number = polygonEntity.mass + circleEntity.mass;
					const inverseTotalMass: number = 1 / totalMass;
					const polygonCorrectionFactor: number = circleEntity.mass * inverseTotalMass;
					const circleCorrectionFactor: number = polygonEntity.mass * inverseTotalMass;
					polygonEntity.moveBy(-correctionX * polygonCorrectionFactor, -correctionY * polygonCorrectionFactor);
					circleEntity.moveBy(correctionX * circleCorrectionFactor, correctionY * circleCorrectionFactor);
				}
				return;
			} else {
				return;
			}
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
			const inverseSumPenetration: number = 1 / sumPenetration;
			weightedNormalX = accumulatedNormalXTimesPenetration * inverseSumPenetration;
			weightedNormalY = accumulatedNormalYTimesPenetration * inverseSumPenetration;
			averageContactX = accumulatedContactXTimesPenetration * inverseSumPenetration;
			averageContactY = accumulatedContactYTimesPenetration * inverseSumPenetration;
		}
		const averageNormalLengthSquared: number = weightedNormalX * weightedNormalX + weightedNormalY * weightedNormalY;
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		if (averageNormalLengthSquared < 1e-18) {
			finalNormalX = 0;
			finalNormalY = 1;
		} else {
			const inverseLength: number = 1 / Math.sqrt(averageNormalLengthSquared);
			finalNormalX = weightedNormalX * inverseLength;
			finalNormalY = weightedNormalY * inverseLength;
		}
		const averagePenetration: number = sumPenetration / contactCount;
		const correctionY: number = finalNormalX * averagePenetration;
		const correctionX: number = finalNormalY * averagePenetration;
		if (polygonEntity.isStatic) {
			circleEntity.moveBy(correctionY, correctionX);
		} else if (circleEntity.isStatic) {
			polygonEntity.moveBy(-correctionY, -correctionX);
		} else {
			const totalMass: number = polygonEntity.mass + circleEntity.mass;
			const inverseTotalMass: number = 1 / totalMass;
			const polygonCorrectionFactor: number = circleEntity.mass * inverseTotalMass;
			const circleCorrectionFactor: number = polygonEntity.mass * inverseTotalMass;
			polygonEntity.moveBy(-correctionY * polygonCorrectionFactor, -correctionX * polygonCorrectionFactor);
			circleEntity.moveBy(correctionY * circleCorrectionFactor, correctionX * circleCorrectionFactor);
		}
		const relativePolygonX: number = averageContactX - polygonEntity.positionX;
		const relativePolygonY: number = averageContactY - polygonEntity.positionY;
		const relativeCircleX: number = averageContactX - circleX;
		const relativeCircleY: number = averageContactY - circleY;
		const polygonVelocityAtContactX: number = polygonEntity.positionalVelocity.x - polygonEntity.angularVelocity * relativePolygonY;
		const polygonVelocityAtContactY: number = polygonEntity.positionalVelocity.y + polygonEntity.angularVelocity * relativePolygonX;
		const circleVelocityContactX: number = circleEntity.positionalVelocity.x - circleEntity.angularVelocity * relativeCircleY;
		const circleVelocityContactY: number = circleEntity.positionalVelocity.y + circleEntity.angularVelocity * relativeCircleX;
		const relativeVelocityX: number = circleVelocityContactX - polygonVelocityAtContactX;
		const relativeVelocityY: number = circleVelocityContactY - polygonVelocityAtContactY;
		const velocityAlongNormal: number = relativeVelocityX * finalNormalX + relativeVelocityY * finalNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inversePolygonMass: number = polygonEntity.isStatic ? 0 : 1 / polygonEntity.mass;
		const inverseCircleMass: number = circleEntity.isStatic ? 0 : 1 / circleEntity.mass;
		let inversePolygonInertia: number = 0;
		if (!polygonEntity.isStatic && polygonEntity.inertia > 1e-9) {
			inversePolygonInertia = 1 / polygonEntity.inertia;
		}
		let inverseCircleInertia: number = 0;
		if (!circleEntity.isStatic && circleEntity.inertia > 1e-9) {
			inverseCircleInertia = 1 / circleEntity.inertia;
		}
		const relativePolygonCrossNormal: number = relativePolygonX * finalNormalY - relativePolygonY * finalNormalX;
		const polygonRotationTerm: number = relativePolygonCrossNormal * relativePolygonCrossNormal * inversePolygonInertia;
		const relativeCircleCrossNormal: number = relativeCircleX * finalNormalY - relativeCircleY * finalNormalX;
		const circleRotationTerm: number = relativeCircleCrossNormal * relativeCircleCrossNormal * inverseCircleInertia;
		const denominator: number = inversePolygonMass + inverseCircleMass + polygonRotationTerm + circleRotationTerm;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = (-2.0 * velocityAlongNormal) / denominator;
		const impulseX: number = impulseMagnitude * finalNormalX;
		const impulseY: number = impulseMagnitude * finalNormalY;
		if (!polygonEntity.isStatic) {
			polygonEntity.positionalVelocity.x -= impulseX * inversePolygonMass;
			polygonEntity.positionalVelocity.y -= impulseY * inversePolygonMass;
			polygonEntity.angularVelocity -= (relativePolygonX * impulseY - relativePolygonY * impulseX) * inversePolygonInertia;
		}

		if (!circleEntity.isStatic) {
			circleEntity.positionalVelocity.x += impulseX * inverseCircleMass;
			circleEntity.positionalVelocity.y += impulseY * inverseCircleMass;
			circleEntity.angularVelocity += (relativeCircleX * impulseY - relativeCircleY * impulseX) * inverseCircleInertia;
		}
	}

	public static collidePolygonPolygon(instance: Entity, other: Entity): void {
		if (instance.isStatic && other.isStatic) {
			return;
		}
		const instancePoints: number[] = instance.points!;
		const otherPoints: number[] = other.points!;
		const instanceTriangles: Triangle[] = triangulate(instancePoints);
		const otherTriangles: Triangle[] = triangulate(otherPoints);
		let bestPenetration: number = 0;
		let bestNormalX: number = 0;
		let bestNormalY: number = 0;
		let bestContactX: number = 0;
		let bestContactY: number = 0;
		let collisionFound: boolean = false;
		for (const instanceTriangle of instanceTriangles) {
			for (const otherTriangle of otherTriangles) {
				const satResult = computeSATConvex(instanceTriangle, otherTriangle);
				if (satResult.collided && satResult.penetration > bestPenetration) {
					bestPenetration = satResult.penetration;
					bestNormalX = satResult.normalX;
					bestNormalY = satResult.normalY;
					const contact = findContactPoint(instanceTriangle, otherTriangle);
					bestContactX = contact.x;
					bestContactY = contact.y;
					collisionFound = true;
				}
			}
		}
		if (!collisionFound) {
			return;
		}
		const correctionX = bestNormalX * bestPenetration;
		const correctionY = bestNormalY * bestPenetration;
		if (instance.isStatic) {
			other.moveBy(correctionX, correctionY);
		} else if (other.isStatic) {
			instance.moveBy(-correctionX, -correctionY);
		} else {
			const totalMass = instance.mass + other.mass;
			const instanceFraction = other.mass / totalMass;
			const otherFraction = instance.mass / totalMass;
			instance.moveBy(-correctionX * instanceFraction, -correctionY * instanceFraction);
			other.moveBy(correctionX * otherFraction, correctionY * otherFraction);
		}
		const relatveInstanceX = bestContactX - instance.positionX;
		const relativeInstanceY = bestContactY - instance.positionY;
		const relativeOtherX = bestContactX - other.positionX;
		const relativeOtherY = bestContactY - other.positionY;
		const instanceVelocityX = instance.positionalVelocity.x - instance.angularVelocity * relativeInstanceY;
		const instanceVelocityY = instance.positionalVelocity.y + instance.angularVelocity * relatveInstanceX;
		const otherVelocityX = other.positionalVelocity.x - other.angularVelocity * relativeOtherY;
		const otherVelocityY = other.positionalVelocity.y + other.angularVelocity * relativeOtherX;
		const relativeVelocityX = otherVelocityX - instanceVelocityX;
		const relativeVelocityY = otherVelocityY - instanceVelocityY;
		const velocityAlongNormal = relativeVelocityX * bestNormalX + relativeVelocityY * bestNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inverseInstanceMass = instance.isStatic ? 0 : 1 / instance.mass;
		const inverseOtherMass = other.isStatic ? 0 : 1 / other.mass;
		let inverseInstanceInertia = 0;
		if (!instance.isStatic && instance.inertia > 1e-9) {
			inverseInstanceInertia = 1 / instance.inertia;
		}
		let inverseOtherInertia = 0;
		if (!other.isStatic && other.inertia > 1e-9) {
			inverseOtherInertia = 1 / other.inertia;
		}
		const instanceCross = relatveInstanceX * bestNormalY - relativeInstanceY * bestNormalX;
		const otherCross = relativeOtherX * bestNormalY - relativeOtherY * bestNormalX;
		const denominator = inverseInstanceMass + inverseOtherMass + instanceCross * instanceCross * inverseInstanceInertia + otherCross * otherCross * inverseOtherInertia;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude = -velocityAlongNormal / denominator;
		const impulseX = impulseMagnitude * bestNormalX;
		const impulseY = impulseMagnitude * bestNormalY;
		if (!instance.isStatic) {
			instance.positionalVelocity.x -= impulseX * inverseInstanceMass;
			instance.positionalVelocity.y -= impulseY * inverseInstanceMass;
		}
		if (!other.isStatic) {
			other.positionalVelocity.x += impulseX * inverseOtherMass;
			other.positionalVelocity.y += impulseY * inverseOtherMass;
		}
		if (!instance.isStatic) {
			const crossImpulse = relatveInstanceX * impulseY - relativeInstanceY * impulseX;
			instance.angularVelocity -= crossImpulse * inverseInstanceInertia;
		}
		if (!other.isStatic) {
			const crossImpulse = relativeOtherX * impulseY - relativeOtherY * impulseX;
			other.angularVelocity += crossImpulse * inverseOtherInertia;
		}
	}

	public static collide(instance: Entity, other: Entity): void {
		if (instance.points === null) {
			if (other.points === null) {
				this.collideCircleCircle(instance, other);
			} else {
				this.collidePolygonCircle(other, instance);
			}
		} else {
			if (other.points === null) {
				this.collidePolygonCircle(instance, other);
			} else {
				this.collidePolygonPolygon(instance, other);
			}
		}
	}
}