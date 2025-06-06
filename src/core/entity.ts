import { computeMEC } from "../geometry/misc";
import { isPointInPolygon } from "../geometry/polygon";
import { Box, Circle, Vector2 } from "../types";

const TAU: number = Math.PI * 2;

export class Entity {
	private static entityIndexTicker: number = 1;
	private _position: Vector2 = { x: 0, y: 0 };
	public positionalVelocity: Vector2 = { x: 0, y: 0 };
	private _angle: number = 0;
	public angularVelocity: number = 0;
	private _mass: number = 1;
	private _inertiaDirty: boolean = true;
	private _inertia: number = 1;
	private _radius: number = 1;
	public isStatic: boolean = false;
	public box: Box = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
	public points: number[] | null = null;
	public frictionCoefficient: number = 1;
	public restitution: number = 0;
	public linearDampingFactor: number = 0.9;
	public angularDampeningFactor: number = 0.9;
	public readonly index: number = Entity.entityIndexTicker++;
	public constructor(positionX: number, positionY: number, radius: number, points: number[] | null = null) {
		this._position.x = positionX;
		this._position.y = positionY;
		this._radius = radius;
		if (points === null) {
			this.box.minX = this._position.x - radius;
			this.box.minY = this._position.y - radius;
			this.box.maxX = this._position.x + radius;
			this.box.maxY = this._position.y + radius;
		} else {
			this.points = [];
			const circle: Circle = computeMEC(points.slice());
			const factor: number = circle.radius < 1e-9 ? 0 : this._radius / circle.radius;
			for (let i: number = 0; i < points.length; i += 2) {
				this.points.push(this._position.x + (points[i] - circle.x) * factor, this._position.y + (points[i + 1] - circle.y) * factor);
			}
			this.updateBox();
		}
	}

	public get positionX(): number {
		return this._position.x;
	}

	public set positionX(x: number) {
		const distance: number = x - this._position.x;
		this._position.x = x;
		this.box.minX += distance;
		this.box.maxX += distance;
		if (this.points === null) {
			return;
		}
		for (let i: number = 0; i < this.points.length; i += 2) {
			this.points[i] += distance;
		}
	}

	public get positionY(): number {
		return this._position.y;
	}

	public set positionY(y: number) {
		const distance: number = y - this._position.y;
		this._position.y = y;
		this.box.minY += distance;
		this.box.maxY += distance;
		if (this.points === null) {
			return;
		}
		for (let i: number = 1; i < this.points.length; i += 2) {
			this.points[i] += distance;
		}
	}

	public get angle(): number {
		return this._angle;
	}

	public set angle(angle: number) {
		const delta: number = angle - this._angle;
		if (angle >= TAU || angle < 0) {
			this._angle = angle - TAU * Math.floor(angle / TAU);
		} else {
			this._angle = angle;
		}
		if (this.points === null) {
			return;
		}
		const cos: number = Math.cos(delta);
		const sin: number = Math.sin(delta);
		for (let i = 0; i < this.points.length; i += 2) {
			const relativeX: number = this.points[i] - this._position.x;
			const relativeY: number = this.points[i + 1] - this._position.y;
			this.points[i] = this._position.x + relativeX * cos - relativeY * sin;
			this.points[i + 1] = this._position.y + relativeX * sin + relativeY * cos;
		}
		this.updateBox();
	}

	public get mass(): number {
		return this._mass;
	}

	public set mass(mass: number) {
		this._mass = mass;
		this._inertiaDirty = true;
	}

	public get radius(): number {
		return this._radius;
	}

	public set radius(radius: number) {
		this._inertiaDirty = true;
		if (this.points === null) {
			this._radius = radius;
			this.box.minX = this._position.x - radius;
			this.box.minY = this._position.y - radius;
			this.box.maxX = this._position.x + radius;
			this.box.maxY = this._position.y + radius;
			return;
		}
		const factor = radius / this._radius;
		this._radius = radius;
		for (let i: number = 0; i < this.points.length; i += 2) {
			this.points[i] = this._position.x + (this.points[i] - this._position.x) * factor;
			this.points[i + 1] = this._position.y + (this.points[i + 1] - this._position.y) * factor;
		}
		this.updateBox();
	}

	public get inertia(): number {
		if (this._inertiaDirty) {
			this._inertiaDirty = false;
			this.updateInertia();
		}
		return this._inertia;
	}

	private updateBox(): void {
		if (this.points === null) {
			this.box.minX = this._position.x - this._radius;
			this.box.minY = this._position.y - this._radius;
			this.box.maxX = this._position.x + this._radius;
			this.box.maxY = this._position.y + this._radius;
		} else {
			this.box.minX = this.points[0];
			this.box.minY = this.points[1];
			this.box.maxX = this.points[0];
			this.box.maxY = this.points[1];
			for (let i: number = 2; i < this.points.length; i += 2) {
				const pointX: number = this.points[i];
				const pointY: number = this.points[i + 1];
				if (this.box.minX > pointX) {
					this.box.minX = pointX;
				}
				if (this.box.minY > pointY) {
					this.box.minY = pointY;
				}
				if (this.box.maxX < pointX) {
					this.box.maxX = pointX;
				}
				if (this.box.maxY < pointY) {
					this.box.maxY = pointY;
				}
			}
		}
	}

	private updateInertia(): void {
		if (this.points === null) {
			this._inertia = 0.5 * this._mass * this._radius * this._radius;
			return;
		}
		let area2Sum: number = 0;
		let momentumSum: number = 0;
		for (let i: number = 0, j: number = this.points.length - 2; i < this.points.length; j = i, i += 2) {
			const currentPointX: number = this.points[i] - this._position.x;
			const currentPointY: number = this.points[i + 1] - this._position.y;
			const previousPointX: number = this.points[j] - this._position.x;
			const previousPointY: number = this.points[j + 1] - this._position.y;
			const cross: number = currentPointX * previousPointY - previousPointX * currentPointY;
			area2Sum += cross;
			momentumSum += cross * (currentPointX * currentPointX + currentPointX * previousPointX + previousPointX * previousPointX + currentPointY * currentPointY + currentPointY * previousPointY + previousPointY * previousPointY);
		}
		this._inertia = this._mass * momentumSum / 6 / area2Sum;
	}

	public moveBy(distanceX: number, distanceY: number): void {
		this._position.x += distanceX;
		this._position.y += distanceY;
		this.box.minX += distanceX;
		this.box.maxX += distanceX;
		this.box.minY += distanceY;
		this.box.maxY += distanceY;
		if (this.points === null) {
			return;
		}
		for (let i = 0; i < this.points.length; i += 2) {
			this.points[i] += distanceX;
			this.points[i + 1] += distanceY;
		}
	}

	public clearForces(): void {
		this.positionalVelocity.x = 0;
		this.positionalVelocity.y = 0;
		this.angularVelocity = 0;
	}

	private applyImpulse(impulse: Vector2, contactPoint: Vector2 | null = null): void {
		this.positionalVelocity.x += impulse.x / this._mass;
		this.positionalVelocity.y += impulse.y / this._mass;
		if (contactPoint === null || this.points === null) {
			return;
		}
		this.angularVelocity += ((contactPoint.x - this._position.x) * impulse.x - (contactPoint.y - this._position.y) * impulse.x) / this.inertia;
	}

	public applyForce(forceX: number, forceY: number, point: Vector2 | null = null): void {
		if (this.isStatic) return;
		this.positionalVelocity.x += forceX / this._mass;
		this.positionalVelocity.y += forceY / this._mass;
		if (point && this.points !== null) {
			const torque = (point.x - this._position.x) * forceY - (point.y - this._position.y) * forceX;
			this.angularVelocity += torque / this.inertia;
		}
	}

	public isPointInside(pointX: number, pointY: number): boolean {
		if (this.points === null) {
			const distanceX = pointX - this._position.x;
			const distanceY = pointY - this._position.y;
			return (distanceX * distanceX + distanceY * distanceY) <= (this._radius * this._radius);
		}
		return isPointInPolygon(pointX, pointY, this.points);
	}

	public clone(): Entity {
		const cloneEntity: Entity = new Entity(this._position.x, this._position.y, this._radius, null);
		cloneEntity.positionalVelocity.x = this.positionalVelocity.x;
		cloneEntity.positionalVelocity.y = this.positionalVelocity.y;
		cloneEntity.angle = this.angle;
		cloneEntity.angularVelocity = this.angularVelocity;
		cloneEntity.isStatic = this.isStatic;
		cloneEntity._mass = this._mass;
		cloneEntity._inertiaDirty = this._inertiaDirty;
		cloneEntity._inertia = this._inertia;
		if (this.points === null) {
			return cloneEntity;
		}
		cloneEntity.points = this.points.slice();
		cloneEntity.box.minX = this.box.minX;
		cloneEntity.box.minY = this.box.minY;
		cloneEntity.box.maxX = this.box.maxX;
		cloneEntity.box.maxY = this.box.maxY;
		return cloneEntity;
	}

	public update(deltaTime: number): void {
		if (Math.abs(this.positionalVelocity.x) > 1e-3) {
			this.positionX += this.positionalVelocity.x * deltaTime;
			this.positionalVelocity.x *= Math.pow(this.linearDampingFactor, deltaTime);
		}
		if (Math.abs(this.positionalVelocity.y) > 1e-3) {
			this.positionY += this.positionalVelocity.y * deltaTime;
			this.positionalVelocity.y *= Math.pow(this.linearDampingFactor, deltaTime);
		}
		if (Math.abs(this.angularVelocity) > 1e-5) {
			this.angle += this.angularVelocity * deltaTime;
			this.angularVelocity *= Math.pow(this.angularDampeningFactor, deltaTime);
		}
	}
}