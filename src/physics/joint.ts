// So currently, the general plan i have rn is to define a set of joint classes that implement the IJoint interface.
// These would be constraints that restrict the relative motion of two bodies, each joint type providing a way to
// control specific behavior such as maintainin a fixed distance, allowing rotation, or simulating different things.

import { IJoint } from "../types";

export class DistanceJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class RevoluteJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class PrismaticJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class PulleyJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class GearJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class WheelJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class WeldJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class RopeJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class FrictionJoint implements IJoint {
	update(deltaTime: number): void {

	}
}

export class MotorJoint implements IJoint {
	update(deltaTime: number): void {

	}
}