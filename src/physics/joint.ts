// So currently, the general plan i have rn is to define a set of joint classes that implement the IJoint interface.
// These would be constraints that restrict the relative motion of two bodies, each joint type providing a way to
// control specific behavior such as maintainin a fixed distance, allowing rotation, or simulating different things.

import { Entity } from "../core/entity";

export abstract class BaseJoint {
	public broken: boolean = false;
	public enabled: boolean = true;
	public appliedForce: number = 0;
	public instance: Entity;
	public other: Entity;
	public breakForce: number;
	constructor(instance: Entity, other: Entity, breakForce: number) {
		this.instance = instance;
		this.other = other;
		this.breakForce = breakForce;
	}

	abstract update(): void;
}

export class DistanceJoint extends BaseJoint {
	public instanceLocalAnchorX: number;
	public instanceLocalAnchorY: number;
	public otherLocalAnchorX: number;
	public otherLocalAnchorY: number;
	public restLength: number;
	public stiffness: number = 1;
	public damping: number = 0.1;
	public minLength: number = 0;
	public maxLength: number = Infinity;

	constructor(instance: Entity, other: Entity, instanceAnchorX: number, instanceAnchorY: number, otherAnchorX: number, otherAnchorY: number, restLength?: number, breakForce: number = Infinity) {
		super(instance, other, breakForce);
		const instanceCos: number = Math.cos(-instance.angle);
		const instanceSin: number = Math.sin(-instance.angle);
		const instanceRelativeX: number = instanceAnchorX - instance.positionX;
		const instanceRelativeY: number = instanceAnchorY - instance.positionY;
		this.instanceLocalAnchorX = instanceRelativeX * instanceCos - instanceRelativeY * instanceSin;
		this.instanceLocalAnchorY = instanceRelativeX * instanceSin + instanceRelativeY * instanceCos;
		const otherCos: number = Math.cos(-other.angle);
		const otherSin: number = Math.sin(-other.angle);
		const otherRelativeX: number = otherAnchorX - other.positionX;
		const otherRelativeY: number = otherAnchorY - other.positionY;
		this.otherLocalAnchorX = otherRelativeX * otherCos - otherRelativeY * otherSin;
		this.otherLocalAnchorY = otherRelativeX * otherSin + otherRelativeY * otherCos;
		if (restLength === undefined) {
			const distanceX: number = instanceAnchorX - otherAnchorX;
			const distanceY: number = instanceAnchorY - otherAnchorY;
			this.restLength = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
		} else {
			this.restLength = restLength;
		}
	}

	public update(): void {
		if (this.broken || !this.enabled) {
			return;
		}
		const instanceCos: number = Math.cos(this.instance.angle);
		const instanceSin: number = Math.sin(this.instance.angle);
		const instanceWorldAnchorX: number = this.instance.positionX + this.instanceLocalAnchorX * instanceCos - this.instanceLocalAnchorY * instanceSin;
		const instanceWorldAnchorY: number = this.instance.positionY + this.instanceLocalAnchorX * instanceSin + this.instanceLocalAnchorY * instanceCos;
		const otherCos: number = Math.cos(this.other.angle);
		const otherSin: number = Math.sin(this.other.angle);
		const otherWorldAnchorX: number = this.other.positionX + this.otherLocalAnchorX * otherCos - this.otherLocalAnchorY * otherSin;
		const otherWorldAnchorY: number = this.other.positionY + this.otherLocalAnchorX * otherSin + this.otherLocalAnchorY * otherCos;
		const distanceX: number = otherWorldAnchorX - instanceWorldAnchorX;
		const distanceY: number = otherWorldAnchorY - instanceWorldAnchorY;
		const distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
		if (distance < 1e-9) {
			return;
		}
		let targetLength: number = this.restLength;
		if (distance < this.minLength) {
			targetLength = this.minLength;
		} else if (distance > this.maxLength) {
			targetLength = this.maxLength;
		} else if (Math.abs(distance - this.restLength) < 1e-6) {
			return;
		}
		const normalX: number = distanceX / distance;
		const normalY: number = distanceY / distance;
		const instanceRelativeX: number = instanceWorldAnchorX - this.instance.positionX;
		const instanceRelativeY: number = instanceWorldAnchorY - this.instance.positionY;
		const otherRelativeX: number = otherWorldAnchorX - this.other.positionX;
		const otherRelativeY: number = otherWorldAnchorY - this.other.positionY;
		const relativeVelocityX: number = (this.other.velocityX - this.other.angularVelocity * otherRelativeY) - (this.instance.velocityX - this.instance.angularVelocity * instanceRelativeY);
		const relativeVelocityY: number = (this.other.velocityY + this.other.angularVelocity * otherRelativeX) - (this.instance.velocityY + this.instance.angularVelocity * instanceRelativeX);
		const inverseInstanceMass: number = this.instance.isStatic ? 0 : 1 / this.instance.mass;
		const inverseOtherMass: number = this.other.isStatic ? 0 : 1 / this.other.mass;
		const inverseInstanceInertia: number = this.instance.isStatic ? 0 : 1 / this.instance.inertia;
		const inverseOtherInertia: number = this.other.isStatic ? 0 : 1 / this.other.inertia;
		const instanceCross: number = instanceRelativeX * normalY - instanceRelativeY * normalX;
		const otherCross: number = otherRelativeX * normalY - otherRelativeY * normalX;
		const effectiveMass = inverseInstanceMass + inverseOtherMass + instanceCross * instanceCross * inverseInstanceInertia + otherCross * otherCross * inverseOtherInertia;
		if (effectiveMass < 1e-9) {
			return;
		}
		const impulse: number = -(this.stiffness * (distance - targetLength) + this.damping * (relativeVelocityX * normalX + relativeVelocityY * normalY)) / effectiveMass;
		const impulseX: number = impulse * normalX;
		const impulseY: number = impulse * normalY;
		if (!this.instance.isStatic) {
			this.instance.velocityX -= impulseX * inverseInstanceMass;
			this.instance.velocityY -= impulseY * inverseInstanceMass;
			this.instance.angularVelocity -= (instanceRelativeX * impulseY - instanceRelativeY * impulseX) * inverseInstanceInertia;
		}
		if (!this.other.isStatic) {
			this.other.velocityX += impulseX * inverseOtherMass;
			this.other.velocityY += impulseY * inverseOtherMass;
			this.other.angularVelocity += (otherRelativeX * impulseY - otherRelativeY * impulseX) * inverseOtherInertia;
		}
		this.appliedForce = Math.abs(impulse);
		if (this.appliedForce > this.breakForce) {
			this.broken = true;
			this.enabled = false;
		}
	}
}

export class RevoluteJoint extends BaseJoint {
	public update(): void {

	}
}

export class PrismaticJoint extends BaseJoint {
	public update(): void {

	}
}

export class PulleyJoint extends BaseJoint {
	public update(): void {

	}
}

export class GearJoint extends BaseJoint {
	public update(): void {

	}
}

export class WheelJoint extends BaseJoint {
	public update(): void {

	}
}

export class WeldJoint extends BaseJoint {
	public update(): void {

	}
}

export class RopeJoint extends BaseJoint {
	public update(): void {

	}
}

export class FrictionJoint extends BaseJoint {
	public update(): void {

	}
}

export class MotorJoint extends BaseJoint {
	public update(): void {

	}
}