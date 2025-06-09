import { Entity } from "../core/entity";
import { addPolygonAxes, findClosestPointOnPolygonToCircle, findContactPoints, isPointInPolygon, projectPolygonOntoAxis } from "../geometry/polygon";
import { ClosestPoint, ContactPoints } from "../types";

export class Collision {
	public static resolve(instance: Entity, other: Entity, normalX: number, normalY: number, penetration: number, contactX: number, contactY: number): void {
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
		const instanceVelocityX: number = instance.velocityX - instance.angularVelocity * relativeInstanceY;
		const instanceVelocityY: number = instance.velocityY + instance.angularVelocity * relativeInstanceX;
		const otherVelocityX: number = other.velocityX - other.angularVelocity * relativeOtherY;
		const otherVelocityY: number = other.velocityY + other.angularVelocity * relativeOtherX;
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
		let restitution = Math.min(instance.restitution, other.restitution);
		if (Math.abs(velocityAlongNormal) < 1e-2) {
			restitution = 0;
		}
		const impulseMagnitude: number = -(1 + restitution) * velocityAlongNormal / impulseDenominator;
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.velocityX -= impulseX * inverseInstanceMass;
			instance.velocityY -= impulseY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * impulseY - relativeInstanceY * impulseX) * inverseInstanceInertia;
		}
		if (!other.isStatic) {
			other.velocityX += impulseX * inverseOtherMass;
			other.velocityY += impulseY * inverseOtherMass;
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
			instance.velocityX -= frictionX * inverseInstanceMass;
			instance.velocityY -= frictionY * inverseInstanceMass;
			instance.angularVelocity -= (relativeInstanceX * frictionY - relativeInstanceY * frictionX) * inverseInstanceInertia;
		}
		if (!other.isStatic) {
			other.velocityX += frictionX * inverseOtherMass;
			other.velocityY += frictionY * inverseOtherMass;
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
		this.resolve(instance, other, normalX, normalY, minimumDistance - distance, instance.positionX + normalX * instance.radius, instance.positionY + normalY * instance.radius);
		return true;
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): boolean {
		const closestPoint: ClosestPoint = findClosestPointOnPolygonToCircle(polygonEntity.points, circleEntity.positionX, circleEntity.positionY);
		if (isPointInPolygon(circleEntity.positionX, circleEntity.positionY, polygonEntity.points)) {
			const distance: number = Math.sqrt(closestPoint.distanceSquared);
			let resolutionNormalX: number = 0;
			let resolutionNormalY: number = 0;
			if (distance < 1e-9) {
				resolutionNormalX = 0;
				resolutionNormalY = 1;
			} else {
				const inverseExactDistance: number = 1 / distance;
				resolutionNormalX = (closestPoint.x - circleEntity.positionX) * inverseExactDistance;
				resolutionNormalY = (closestPoint.y - circleEntity.positionY) * inverseExactDistance;
			}
			const totalPenetrationDistance: number = circleEntity.radius + distance;
			this.resolve(polygonEntity, circleEntity, resolutionNormalX, resolutionNormalY, totalPenetrationDistance, closestPoint.x, closestPoint.y);
			return true;
		}
		if (closestPoint.distanceSquared >= circleEntity.radius * circleEntity.radius) {
			return false;
		}
		const distance: number = Math.sqrt(closestPoint.distanceSquared);
		let resolutionNormalX: number = 0;
		let resolutionNormalY: number = 0;
		if (distance < 1e-9) {
			resolutionNormalX = 0;
			resolutionNormalY = 1;
		} else {
			const inverseDistance: number = 1 / distance;
			resolutionNormalX = (circleEntity.positionX - closestPoint.x) * inverseDistance;
			resolutionNormalY = (circleEntity.positionY - closestPoint.y) * inverseDistance;
		}
		this.resolve(polygonEntity, circleEntity, resolutionNormalX, resolutionNormalY, circleEntity.radius - distance, closestPoint.x, closestPoint.y);
		return true;
	}

	public static collidePolygonPolygon(instanceEntity: Entity, otherEntity: Entity): boolean {
		let minimumOverlap: number = Infinity;
		let overlapNormalX: number = 0;
		let overlapNormalY: number = 0;
		const combinedAxes: number[] = [];
		addPolygonAxes(instanceEntity.points, combinedAxes);
		addPolygonAxes(otherEntity.points, combinedAxes);
		for (let i: number = 0; i < combinedAxes.length; i += 2) {
			const currentX: number = combinedAxes[i];
			const currentY: number = combinedAxes[i + 1];
			const instanceProjectionResult = projectPolygonOntoAxis(instanceEntity.points, currentX, currentY);
			const otherProjectionResult = projectPolygonOntoAxis(otherEntity.points, currentX, currentY);
			if (instanceProjectionResult.maximum < otherProjectionResult.minimum || otherProjectionResult.maximum < instanceProjectionResult.minimum) {
				return false;
			}
			const otherOverlap: number = instanceProjectionResult.maximum - otherProjectionResult.minimum;
			const instanceOverlap: number = otherProjectionResult.maximum - instanceProjectionResult.minimum;
			const overlap: number = otherOverlap < instanceOverlap ? otherOverlap : instanceOverlap;
			if (overlap < minimumOverlap) {
				minimumOverlap = overlap;
				const centerToCenterProjection: number = (otherEntity.positionX - instanceEntity.positionX) * currentX + (otherEntity.positionY - instanceEntity.positionY) * currentY;
				if (centerToCenterProjection < 0) {
					overlapNormalX = -currentX;
					overlapNormalY = -currentY;
				} else {
					overlapNormalX = currentX;
					overlapNormalY = currentY;
				}
			}
		}
		if (minimumOverlap < 1e-9) {
			return false;
		}
		const contactPointResult: ContactPoints = findContactPoints(instanceEntity.points, otherEntity.points);
		let finalContactX: number = contactPointResult.x;
		let finalContactY: number = contactPointResult.y;
		if (contactPointResult.count === 0) {
			finalContactX = (instanceEntity.positionX + otherEntity.positionX) * 0.5;
			finalContactY = (instanceEntity.positionY + otherEntity.positionY) * 0.5;
		}
		this.resolve(instanceEntity, otherEntity, overlapNormalX, overlapNormalY, minimumOverlap, finalContactX, finalContactY);
		return true;
	}

	public static collide(instance: Entity, other: Entity): boolean {
		if (instance.isCircle) {
			if (other.isCircle) {
				return this.collideCircleCircle(instance, other);
			}
			return this.collidePolygonCircle(other, instance);
		}
		if (other.isCircle) {
			return this.collidePolygonCircle(instance, other);
		}
		return this.collidePolygonPolygon(instance, other);
	}
}