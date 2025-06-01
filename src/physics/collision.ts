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
			const inverseTotalMass: number = 1 / (instance.mass + other.mass);
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
			other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const instanceInverseMass: number = 1 / instance.mass;
		const otherInverseMass: number = 1 / other.mass;
		const impulseMagnitude: number = -2 * (relativeVelocityX * normalX + relativeVelocityY * normalY) * (1 / (instanceInverseMass + otherInverseMass));
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