import { Entity } from "../entity/entity";

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared === 0 || distanceSquared >= minimumDistance * minimumDistance) {
			return;
		}
		if (instance.isStatic && other.isStatic) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		const inverseDistance: number = 1 / distance;
		const normalX: number = distanceX * inverseDistance;
		const normalY: number = distanceY * inverseDistance;
		const overlap: number = minimumDistance - distance;
		if (instance.isStatic) {
			other.moveBy(normalX * overlap, normalY * overlap);
		} else if (other.isStatic) {
			instance.moveBy(-normalX * overlap, -normalY * overlap);
		} else {
			const totalMass: number = instance.mass + other.mass;
			if (totalMass > 0) {
				const invTotalMass: number = 1 / totalMass;
				const instanceCorrectionFraction: number = other.mass * invTotalMass;
				const otherCorrectionFraction: number = 1 - instanceCorrectionFraction;
				instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
				other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
			} else {
				const halfOverlap: number = overlap * 0.5;
				instance.moveBy(-normalX * halfOverlap, -normalY * halfOverlap);
				other.moveBy(normalX * halfOverlap, normalY * halfOverlap);
			}
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const otherInverseMass: number = other.isStatic ? 0 : 1 / other.mass;
		if (instanceInverseMass === 0 && otherInverseMass === 0) {
			return;
		}
		const impulseDenominator: number = instanceInverseMass + otherInverseMass;
		const invImpulseDenominator: number = 1 / impulseDenominator;
		const impulseMagnitude: number = -2 * velocityAlongNormal * invImpulseDenominator;
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

	public static collide(instance: Entity, other: Entity): void {
		this.collideCircleCircle(instance, other);
	}
}