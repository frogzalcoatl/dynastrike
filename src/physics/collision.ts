import { Entity } from "../entity/entity";
import { Vector2 } from "../geometry/vector";
import {
	computePenetrationAndNormal,
	computeSATConvex,
	ContactResult,
	findContactPoint,
	isPointInPolygon,
	Projection,
	projectPointOnEdge,
	Triangle,
	triangulate
} from "./utilities";

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		if (instance.isStatic && other.isStatic) {
			return;
		}
		const distanceX: number = other.positionX - instance.positionX;
		const distanceY: number = other.positionY - instance.positionY;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		const minimumDistanceSquared: number = minimumDistance * minimumDistance;
		if (distanceSquared >= minimumDistanceSquared || (instance.isStatic && other.isStatic)) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		let normalX: number;
		let normalY: number;
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
			const inverseTotalMass: number = 1 / (instance.mass + other.mass);
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-overlapCorrectionX * instanceCorrectionFraction, -overlapCorrectionY * instanceCorrectionFraction);
			other.moveBy(overlapCorrectionX * otherCorrectionFraction, overlapCorrectionY * otherCorrectionFraction);
		}
		const velocityAlongNormal: number = (other.positionalVelocity.x - instance.positionalVelocity.x) * normalX + (other.positionalVelocity.y - instance.positionalVelocity.y) * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const otherInverseMass: number = other.isStatic ? 0 : 1 / other.mass;
		const sumInverseMass: number = instanceInverseMass + otherInverseMass;
		if (sumInverseMass < 1e-9) {
			return;
		}
		const instanceInverseInertia: number = instance.isStatic || instance.inertia <= 1e-9 ? 0 : 1 / instance.inertia;
		const otherInverseInertia: number = other.isStatic || other.inertia <= 1e-9 ? 0 : 1 / other.inertia;
		const impulseMagnitude: number = -(1 + Math.min(instance.restitution, other.restitution)) * velocityAlongNormal / sumInverseMass;
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		instance.positionalVelocity.x -= impulseX * instanceInverseMass;
		instance.positionalVelocity.y -= impulseY * instanceInverseMass;
		other.positionalVelocity.x += impulseX * otherInverseMass;
		other.positionalVelocity.y += impulseY * otherInverseMass;
		const tangentX: number = -normalY;
		const tangentY: number = normalX;
		const relativeInstanceX: number = normalX * instance.radius;
		const relativeInstanceY: number = normalY * instance.radius;
		const relativeOtherX: number = -normalX * other.radius;
		const relativeOtherY: number = -normalY * other.radius;
		const frictionDenominator: number = sumInverseMass + (relativeInstanceX * tangentY - relativeInstanceY * tangentX) * (relativeInstanceX * tangentY - relativeInstanceY * tangentX) * instanceInverseInertia + (relativeOtherX * tangentY - relativeOtherY * tangentX) * (relativeOtherX * tangentY - relativeOtherY * tangentX) * otherInverseInertia;
		if (frictionDenominator < 1e-9) {
			return;
		}
		let frictionMagnitude = -((other.positionalVelocity.x - other.angularVelocity * relativeOtherY - instance.positionalVelocity.x - instance.angularVelocity * relativeInstanceY) * tangentX + (other.positionalVelocity.y + other.angularVelocity * relativeOtherX - instance.positionalVelocity.y + instance.angularVelocity * relativeInstanceX) * tangentY) / frictionDenominator;
		const maxFriction = Math.sqrt(instance.frictionCoefficient * other.frictionCoefficient) * Math.abs(impulseMagnitude);
		if (frictionMagnitude > maxFriction) {
			frictionMagnitude = maxFriction;
		} else if (frictionMagnitude < -maxFriction) {
			frictionMagnitude = -maxFriction;
		}
		const frictionX: number = frictionMagnitude * tangentX;
		const frictionY: number = frictionMagnitude * tangentY;
		instance.positionalVelocity.x -= frictionX * instanceInverseMass;
		instance.positionalVelocity.y -= frictionY * instanceInverseMass;
		other.positionalVelocity.x += frictionX * otherInverseMass;
		other.positionalVelocity.y += frictionY * otherInverseMass;
		instance.angularVelocity -= (relativeInstanceX * frictionY - relativeInstanceY * frictionX) * instanceInverseInertia;
		other.angularVelocity += (relativeOtherX * frictionY - relativeOtherY * frictionX) * otherInverseInertia;
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		if (polygonEntity.points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleRadiusSquared: number = circleEntity.radius * circleEntity.radius;
		const isInside: boolean = isPointInPolygon(circleEntity.positionX, circleEntity.positionY, polygonEntity.points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedContactXTimesPenetration: number = 0;
		let accumulatedContactYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let minimumDistanceSqForInside: number = Infinity;
		let featureXForInsideNormal: number = 0;
		let featureYForInsideNormal: number = 0;
		const pointCount: number = polygonEntity.points.length;
		let previousVertexX: number = polygonEntity.points[pointCount - 2];
		let previousVertexY: number = polygonEntity.points[pointCount - 1];
		for (let i = 0; i < pointCount; i += 2) {
			const currentVertexX: number = polygonEntity.points[i];
			const currentVertexY: number = polygonEntity.points[i + 1];
			const projection: Projection = projectPointOnEdge(circleEntity.positionX, circleEntity.positionY, previousVertexX, previousVertexY, currentVertexX, currentVertexY);
			const edgeContact: ContactResult = computePenetrationAndNormal(circleEntity.positionX, circleEntity.positionY, projection.x, projection.y, circleEntity.radius, polygonEntity.positionX, polygonEntity.positionY, isInside, projection.distanceSquared, circleRadiusSquared);
			if (edgeContact.penetration > 0) {
				sumPenetration += edgeContact.penetration;
				accumulatedNormalXTimesPenetration += edgeContact.normalX * edgeContact.penetration;
				accumulatedNormalYTimesPenetration += edgeContact.normalY * edgeContact.penetration;
				accumulatedContactXTimesPenetration += projection.x * edgeContact.penetration;
				accumulatedContactYTimesPenetration += projection.y * edgeContact.penetration;
				contactCount++;
			}
			if (isInside && projection.distanceSquared < minimumDistanceSqForInside) {
				minimumDistanceSqForInside = projection.distanceSquared;
				featureXForInsideNormal = projection.x;
				featureYForInsideNormal = projection.y;
			}
			const distVertexCircleX: number = circleEntity.positionX - currentVertexX;
			const distVertexCircleY: number = circleEntity.positionY - currentVertexY;
			const vertexDistanceSquared: number = distVertexCircleX * distVertexCircleX + distVertexCircleY * distVertexCircleY;
			const vertexContact: ContactResult = computePenetrationAndNormal(circleEntity.positionX, circleEntity.positionY, currentVertexX, currentVertexY, circleEntity.radius, polygonEntity.positionX, polygonEntity.positionY, isInside, vertexDistanceSquared, circleRadiusSquared);
			if (vertexContact.penetration > 0) {
				sumPenetration += vertexContact.penetration;
				accumulatedNormalXTimesPenetration += vertexContact.normalX * vertexContact.penetration;
				accumulatedNormalYTimesPenetration += vertexContact.normalY * vertexContact.penetration;
				accumulatedContactXTimesPenetration += currentVertexX * vertexContact.penetration;
				accumulatedContactYTimesPenetration += currentVertexY * vertexContact.penetration;
				contactCount++;
			}
			if (isInside && vertexDistanceSquared < minimumDistanceSqForInside) {
				minimumDistanceSqForInside = vertexDistanceSquared;
				featureXForInsideNormal = currentVertexX;
				featureYForInsideNormal = currentVertexY;
			}
			previousVertexX = currentVertexX;
			previousVertexY = currentVertexY;
		}
		if (contactCount === 0) {
			if (!isInside || minimumDistanceSqForInside === Infinity) {
				return;
			}
			const distanceToBoundary: number = Math.sqrt(minimumDistanceSqForInside);
			let bestNormalX: number;
			let bestNormalY: number;
			if (distanceToBoundary < 1e-9) {
				bestNormalX = 0;
				bestNormalY = 1;
			} else {
				const inverseDistance: number = 1 / distanceToBoundary;
				bestNormalX = (featureXForInsideNormal - circleEntity.positionX) * inverseDistance;
				bestNormalY = (featureYForInsideNormal - circleEntity.positionY) * inverseDistance;
			}
			const penetration: number = circleEntity.radius + distanceToBoundary;
			const correctionX: number = bestNormalX * penetration;
			const correctionY: number = bestNormalY * penetration;
			if (polygonEntity.isStatic) {
				circleEntity.moveBy(correctionX, correctionY);
			} else if (circleEntity.isStatic) {
				polygonEntity.moveBy(-correctionX, -correctionY);
			} else {
				const invTotalMass: number = 1 / (polygonEntity.mass + circleEntity.mass);
				const polygonCorrectionFactor: number = circleEntity.mass * invTotalMass;
				const circleCorrectionFactor: number = polygonEntity.mass * invTotalMass;
				polygonEntity.moveBy(-correctionX * polygonCorrectionFactor, -correctionY * polygonCorrectionFactor);
				circleEntity.moveBy(correctionX * circleCorrectionFactor, correctionY * circleCorrectionFactor);
			}
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
			const inverseSumPenetration: number = 1 / sumPenetration;
			weightedNormalX = accumulatedNormalXTimesPenetration * inverseSumPenetration;
			weightedNormalY = accumulatedNormalYTimesPenetration * inverseSumPenetration;
			averageContactX = accumulatedContactXTimesPenetration * inverseSumPenetration;
			averageContactY = accumulatedContactYTimesPenetration * inverseSumPenetration;
		}
		const averageLengthSquared: number = weightedNormalX * weightedNormalX + weightedNormalY * weightedNormalY;
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		if (averageLengthSquared < 1e-18) {
			finalNormalX = 0;
			finalNormalY = 1;
		} else {
			const inverseLength: number = 1 / Math.sqrt(averageLengthSquared);
			finalNormalX = weightedNormalX * inverseLength;
			finalNormalY = weightedNormalY * inverseLength;
		}
		const averagePenetration: number = sumPenetration / contactCount;
		const correctionX: number = finalNormalX * averagePenetration;
		const correctionY: number = finalNormalY * averagePenetration;
		let polygonFactor: number = 0;
		let circleFactor: number = 0;
		if (polygonEntity.isStatic) {
			polygonFactor = 0;
			circleFactor = 1;
		} else if (circleEntity.isStatic) {
			polygonFactor = 1;
			circleFactor = 0;
		} else {
			const inverseTotalMass: number = 1 / (polygonEntity.mass + circleEntity.mass);
			polygonFactor = circleEntity.mass * inverseTotalMass;
			circleFactor = polygonEntity.mass * inverseTotalMass;
		}
		polygonEntity.moveBy(-correctionX * polygonFactor, -correctionY * polygonFactor);
		circleEntity.moveBy(correctionX * circleFactor, correctionY * circleFactor);
		const polygonRelativeX: number = averageContactX - polygonEntity.positionX;
		const polygonRelativeY: number = averageContactY - polygonEntity.positionY;
		const circleRelativeX: number = averageContactX - circleEntity.positionX;
		const circleRelativeY: number = averageContactY - circleEntity.positionY;
		const deltaVelocityX = circleEntity.positionalVelocity.x - circleEntity.angularVelocity * circleRelativeY - polygonEntity.positionalVelocity.x + polygonEntity.angularVelocity * polygonRelativeY;
		const deltaVelocityY = circleEntity.positionalVelocity.y + circleEntity.angularVelocity * circleRelativeX - polygonEntity.positionalVelocity.y - polygonEntity.angularVelocity * polygonRelativeX;
		const velocityAlongNormal: number = deltaVelocityX * finalNormalX + deltaVelocityY * finalNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		let inversePolygonMass: number = 0;
		let inversePolygonInertia: number = 0;
		if (!polygonEntity.isStatic) {
			inversePolygonMass = 1 / polygonEntity.mass;
			inversePolygonInertia = 1 / polygonEntity.inertia;
		}
		let inverseCircleMass: number = 0;
		let inverseCircleInertia: number = 0;
		if (!circleEntity.isStatic) {
			inverseCircleMass = 1 / circleEntity.mass;
			inverseCircleInertia = 1 / circleEntity.inertia;
		}
		const polygonCrossNormal: number = polygonRelativeX * finalNormalY - polygonRelativeY * finalNormalX;
		const circleCrossNormal: number = circleRelativeX * finalNormalY - circleRelativeY * finalNormalX;
		const denominator: number = inversePolygonMass + inverseCircleMass + polygonCrossNormal * polygonCrossNormal * inversePolygonInertia + circleCrossNormal * circleCrossNormal * inverseCircleInertia;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = (-(1 + Math.min(polygonEntity.restitution, circleEntity.restitution)) * velocityAlongNormal) / denominator;
		const impulseX: number = impulseMagnitude * finalNormalX;
		const impulseY: number = impulseMagnitude * finalNormalY;
		polygonEntity.positionalVelocity.x -= impulseX * inversePolygonMass;
		polygonEntity.positionalVelocity.y -= impulseY * inversePolygonMass;
		polygonEntity.angularVelocity -= (polygonRelativeX * impulseY - polygonRelativeY * impulseX) * inversePolygonInertia;
		circleEntity.positionalVelocity.x += impulseX * inverseCircleMass;
		circleEntity.positionalVelocity.y += impulseY * inverseCircleMass;
		circleEntity.angularVelocity += (circleRelativeX * impulseY - circleRelativeY * impulseX) * inverseCircleInertia;
		const tangentX: number = -finalNormalY;
		const tangentY: number = finalNormalX;
		const polygonCrossTangent: number = polygonRelativeX * tangentY - polygonRelativeY * tangentX;
		const circleCrossTangent: number = circleRelativeX * tangentY - circleRelativeY * tangentX;
		const frictionDenom: number = inversePolygonMass + inverseCircleMass + polygonCrossTangent * polygonCrossTangent * inversePolygonInertia + circleCrossTangent * circleCrossTangent * inverseCircleInertia;
		if (frictionDenom < 1e-9) {
			return;
		}
		let frictionMagnitude: number = -(deltaVelocityX * tangentX + deltaVelocityY * tangentY) / frictionDenom;
		const maxFriction: number = Math.sqrt(polygonEntity.frictionCoefficient * circleEntity.frictionCoefficient) * Math.abs(impulseMagnitude);
		if (frictionMagnitude > maxFriction) {
			frictionMagnitude = maxFriction;
		} else if (frictionMagnitude < -maxFriction) {
			frictionMagnitude = -maxFriction;
		}
		const frictionX: number = frictionMagnitude * tangentX;
		const frictionY: number = frictionMagnitude * tangentY;
		polygonEntity.positionalVelocity.x -= frictionX * inversePolygonMass;
		polygonEntity.positionalVelocity.y -= frictionY * inversePolygonMass;
		polygonEntity.angularVelocity -= (polygonRelativeX * frictionY - polygonRelativeY * frictionX) * inversePolygonInertia;
		circleEntity.positionalVelocity.x += frictionX * inverseCircleMass;
		circleEntity.positionalVelocity.y += frictionY * inverseCircleMass;
		circleEntity.angularVelocity += (circleRelativeX * frictionY - circleRelativeY * frictionX) * inverseCircleInertia;
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
					const contact: Vector2 = findContactPoint(instanceTriangle, otherTriangle);
					bestContactX = contact.x;
					bestContactY = contact.y;
					collisionFound = true;
				}
			}
		}
		if (!collisionFound) {
			return;
		}
		const correctionX: number = bestNormalX * bestPenetration;
		const correctionY: number = bestNormalY * bestPenetration;
		const instanceIsStatic: boolean = instance.isStatic;
		const otherIsStatic: boolean = other.isStatic;
		if (instanceIsStatic) {
			other.moveBy(correctionX, correctionY);
		} else if (otherIsStatic) {
			instance.moveBy(-correctionX, -correctionY);
		} else {
			const totalMass: number = instance.mass + other.mass;
			const instanceFraction: number = other.mass / totalMass;
			const otherFraction: number = instance.mass / totalMass;
			instance.moveBy(-correctionX * instanceFraction, -correctionY * instanceFraction);
			other.moveBy(correctionX * otherFraction, correctionY * otherFraction);
		}
		const relativeInstanceX: number = bestContactX - instance.positionX;
		const relativeInstanceY: number = bestContactY - instance.positionY;
		const relativeOtherX: number = bestContactX - other.positionX;
		const relativeOtherY: number = bestContactY - other.positionY;
		const instanceVelocityX: number = instance.positionalVelocity.x - instance.angularVelocity * relativeInstanceY;
		const instanceVelocityY: number = instance.positionalVelocity.y + instance.angularVelocity * relativeInstanceX;
		const otherVelocityX: number = other.positionalVelocity.x - other.angularVelocity * relativeOtherY;
		const otherVelocityY: number = other.positionalVelocity.y + other.angularVelocity * relativeOtherX;
		const relativeVelocityX: number = otherVelocityX - instanceVelocityX;
		const relativeVelocityY: number = otherVelocityY - instanceVelocityY;
		const velocityAlongNormal: number = relativeVelocityX * bestNormalX + relativeVelocityY * bestNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inverseInstanceMass: number = instanceIsStatic ? 0 : 1 / instance.mass;
		const inverseOtherMass: number = otherIsStatic ? 0 : 1 / other.mass;
		const inverseInstanceInertia: number = (!instanceIsStatic && instance.inertia > 1e-9) ? 1 / instance.inertia : 0;
		const inverseOtherInertia: number = (!otherIsStatic && other.inertia > 1e-9) ? 1 / other.inertia : 0;
		const instanceCross: number = relativeInstanceX * bestNormalY - relativeInstanceY * bestNormalX;
		const otherCross: number = relativeOtherX * bestNormalY - relativeOtherY * bestNormalX;
		const denominator: number = inverseInstanceMass + inverseOtherMass + instanceCross * instanceCross * inverseInstanceInertia + otherCross * otherCross * inverseOtherInertia;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = -(1 + Math.min(instance.restitution, other.restitution)) * velocityAlongNormal / denominator;
		const impulseX: number = impulseMagnitude * bestNormalX;
		const impulseY: number = impulseMagnitude * bestNormalY;
		if (!instanceIsStatic) {
			instance.positionalVelocity.x -= impulseX * inverseInstanceMass;
			instance.positionalVelocity.y -= impulseY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * impulseY - relativeInstanceY * impulseX) * inverseInstanceInertia;
		}
		if (!otherIsStatic) {
			other.positionalVelocity.x += impulseX * inverseOtherMass;
			other.positionalVelocity.y += impulseY * inverseOtherMass;
			other.angularVelocity += (relativeOtherX * impulseY - relativeOtherY * impulseX) * inverseOtherInertia;
		}
		const tangentX: number = -bestNormalY;
		const tangentY: number = bestNormalX;
		const relVelTangent: number = relativeVelocityX * tangentX + relativeVelocityY * tangentY;
		const instanceCrossTangent: number = relativeInstanceX * tangentY - relativeInstanceY * tangentX;
		const otherCrossTangent: number = relativeOtherX * tangentY - relativeOtherY * tangentX;
		const frictionDenominator: number = inverseInstanceMass + inverseOtherMass + instanceCrossTangent * instanceCrossTangent * inverseInstanceInertia + otherCrossTangent * otherCrossTangent * inverseOtherInertia;
		if (frictionDenominator < 1e-9) {
			return;
		}
		let frictionMagnitude: number = -relVelTangent / frictionDenominator;
		const maxFriction: number = Math.sqrt(instance.frictionCoefficient * other.frictionCoefficient) * Math.abs(impulseMagnitude);
		if (frictionMagnitude > maxFriction) {
			frictionMagnitude = maxFriction;
		} else if (frictionMagnitude < -maxFriction) {
			frictionMagnitude = -maxFriction;
		}
		const frictionX: number = frictionMagnitude * tangentX;
		const frictionY: number = frictionMagnitude * tangentY;
		if (!instanceIsStatic) {
			instance.positionalVelocity.x -= frictionX * inverseInstanceMass;
			instance.positionalVelocity.y -= frictionY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * frictionY - relativeInstanceY * frictionX) * inverseInstanceInertia;
		}
		if (!otherIsStatic) {
			other.positionalVelocity.x += frictionX * inverseOtherMass;
			other.positionalVelocity.y += frictionY * inverseOtherMass;
			other.angularVelocity += (relativeOtherX * frictionY - relativeOtherY * frictionX) * inverseOtherInertia;
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
