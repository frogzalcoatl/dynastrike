import { Entity } from "../core/entity";
import { computePenetrationAndNormal, findContactPoint, projectPointOnEdge } from "../geometry/misc";
import { isPointInPolygon, triangulate } from "../geometry/polygon";
import { ContactResult, Projection, Triangle, Vector2 } from "../types";
import { computeSATConvex } from "./sat";

export class Collision {
	private static resolve(instance: Entity, other: Entity, normalX: number, normalY: number, penetration: number, contactX: number, contactY: number): void {
		const overlapCorrectionX: number = normalX * penetration;
		const overlapCorrectionY: number = normalY * penetration;
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
		const relativeInstanceX: number = contactX - instance.positionX;
		const relativeInstanceY: number = contactY - instance.positionY;
		const relativeOtherX: number = contactX - other.positionX;
		const relativeOtherY: number = contactY - other.positionY;
		const instanceVelocityX: number = instance.positionalVelocity.x - instance.angularVelocity * relativeInstanceY;
		const instanceVelocityY: number = instance.positionalVelocity.y + instance.angularVelocity * relativeInstanceX;
		const otherVelocityX: number = other.positionalVelocity.x - other.angularVelocity * relativeOtherY;
		const otherVelocityY: number = other.positionalVelocity.y + other.angularVelocity * relativeOtherX;
		const relativeVelocityX: number = otherVelocityX - instanceVelocityX;
		const relativeVelocityY: number = otherVelocityY - instanceVelocityY;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inverseInstanceMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const inverseOtherMass: number = other.isStatic ? 0 : 1 / other.mass;
		const inverseInstanceInertia: number = (!instance.isStatic && instance.inertia > 1e-9) ? 1 / instance.inertia : 0;
		const inverseOtherInertia: number = (!other.isStatic && other.inertia > 1e-9) ? 1 / other.inertia : 0;
		const instanceCrossNormal: number = relativeInstanceX * normalY - relativeInstanceY * normalX;
		const otherCrossNormal: number = relativeOtherX * normalY - relativeOtherY * normalX;
		const impulseDenominator: number = inverseInstanceMass + inverseOtherMass + instanceCrossNormal * instanceCrossNormal * inverseInstanceInertia + otherCrossNormal * otherCrossNormal * inverseOtherInertia;
		if (impulseDenominator < 1e-9) {
			return;
		}
		const restitution: number = Math.min(instance.restitution, other.restitution);
		const impulseMagnitude: number = -(1 + restitution) * velocityAlongNormal / impulseDenominator;
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.positionalVelocity.x -= impulseX * inverseInstanceMass;
			instance.positionalVelocity.y -= impulseY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * impulseY - relativeInstanceY * impulseX) * inverseInstanceInertia;
		}
		if (!other.isStatic) {
			other.positionalVelocity.x += impulseX * inverseOtherMass;
			other.positionalVelocity.y += impulseY * inverseOtherMass;
			other.angularVelocity += (relativeOtherX * impulseY - relativeOtherY * impulseX) * inverseOtherInertia;
		}
		const tangentX: number = -normalY;
		const tangentY: number = normalX;
		const relativeVelocityTangent: number = relativeVelocityX * tangentX + relativeVelocityY * tangentY;
		const instanceCrossTangent: number = relativeInstanceX * tangentY - relativeInstanceY * tangentX;
		const otherCrossTangent: number = relativeOtherX * tangentY - relativeOtherY * tangentX;
		const frictionDenominator: number = inverseInstanceMass + inverseOtherMass + instanceCrossTangent * instanceCrossTangent * inverseInstanceInertia + otherCrossTangent * otherCrossTangent * inverseOtherInertia;
		if (frictionDenominator < 1e-9) {
			return;
		}
		let frictionMagnitude: number = -relativeVelocityTangent / frictionDenominator;
		const maxFriction: number = Math.sqrt(instance.frictionCoefficient * other.frictionCoefficient) * Math.abs(impulseMagnitude);
		if (frictionMagnitude > maxFriction) {
			frictionMagnitude = maxFriction;
		} else if (frictionMagnitude < -maxFriction) {
			frictionMagnitude = -maxFriction;
		}
		const frictionX: number = frictionMagnitude * tangentX;
		const frictionY: number = frictionMagnitude * tangentY;
		if (!instance.isStatic) {
			instance.positionalVelocity.x -= frictionX * inverseInstanceMass;
			instance.positionalVelocity.y -= frictionY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * frictionY - relativeInstanceY * frictionX) * inverseInstanceInertia;
		}
		if (!other.isStatic) {
			other.positionalVelocity.x += frictionX * inverseOtherMass;
			other.positionalVelocity.y += frictionY * inverseOtherMass;
			other.angularVelocity += (relativeOtherX * frictionY - relativeOtherY * frictionX) * inverseOtherInertia;
		}
	}

	public static collideCircleCircle(instance: Entity, other: Entity): boolean {
		const distanceX: number = other.positionX - instance.positionX;
		const distanceY: number = other.positionY - instance.positionY;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance) {
			return false;
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
		this.resolve(instance, other, normalX, normalY, minimumDistance - distance, instance.positionX + normalX * instance.radius, instance.positionY + normalY * instance.radius);
		return true;
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): boolean {
		if (polygonEntity.points === null) {
			return false;
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
		for (let i: number = 0; i < pointCount; i += 2) {
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
			const distanceVertexCircleX: number = circleEntity.positionX - currentVertexX;
			const distanceVertexCircleY: number = circleEntity.positionY - currentVertexY;
			const vertexDistanceSquared: number = distanceVertexCircleX * distanceVertexCircleX + distanceVertexCircleY * distanceVertexCircleY;
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
				return false;
			}
			const distanceToBoundary: number = Math.sqrt(minimumDistanceSqForInside);
			let bestNormalX: number = 0;
			let bestNormalY: number = 0;
			if (distanceToBoundary < 1e-9) {
				bestNormalX = 0;
				bestNormalY = 1;
			} else {
				const inverseDistance: number = 1 / distanceToBoundary;
				bestNormalX = (featureXForInsideNormal - circleEntity.positionX) * inverseDistance;
				bestNormalY = (featureYForInsideNormal - circleEntity.positionY) * inverseDistance;
			}
			const collisionPenetration: number = circleEntity.radius + distanceToBoundary;
			this.resolve(polygonEntity, circleEntity, bestNormalX, bestNormalY, collisionPenetration, featureXForInsideNormal, featureYForInsideNormal);
			return true;
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
		this.resolve(polygonEntity, circleEntity, finalNormalX, finalNormalY, sumPenetration / contactCount, averageContactX, averageContactY);
		return true;
	}

	public static collidePolygonPolygon(instance: Entity, other: Entity): boolean {
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
			return false;
		}
		this.resolve(instance, other, bestNormalX, bestNormalY, bestPenetration, bestContactX, bestContactY);
		return true;
	}

	public static collide(instance: Entity, other: Entity): boolean {
		if (instance.points === null) {
			if (other.points === null) {
				return this.collideCircleCircle(instance, other);
			}
			return this.collidePolygonCircle(other, instance);
		}
		if (other.points === null) {
			return this.collidePolygonCircle(instance, other);
		}
		return this.collidePolygonPolygon(instance, other);
	}
}