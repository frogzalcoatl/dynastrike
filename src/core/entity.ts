import { computeMEC } from "../geometry/misc";
import { isPointInPolygon } from "../geometry/polygon";
import { Circle, SerializedEntity } from "../types";

const TAU: number = Math.PI * 2;

export class Entity {
	public static entityIndexTicker: number = 1;

	public static serialize(entity: Entity): SerializedEntity {
		return {
			positionX: entity.positionX,
			positionY: entity.positionY,
			velocityX: entity.velocityX,
			velocityY: entity.velocityY,
			radius: entity.radius,
			points: entity.points?.slice() ?? null,
			angle: entity.angle,
			angularVelocity: entity.angularVelocity,
			mass: entity.mass,
			isStatic: entity.isStatic,
			frictionCoefficient: entity.frictionCoefficient,
			restitution: entity.restitution,
			linearDampingFactor: entity.linearDampingFactor,
			angularDampingFactor: entity.angularDampingFactor
		};
	}

	public static deserialize(data: SerializedEntity): Entity {
		const entity = new Entity(data.positionX, data.positionY, data.radius, data.points);
		entity.angle = data.angle;
		entity.velocityX = data.velocityX;
		entity.velocityY = data.velocityY;
		entity.angularVelocity = data.angularVelocity;
		entity.mass = data.mass;
		entity.isStatic = data.isStatic;
		entity.frictionCoefficient = data.frictionCoefficient;
		entity.restitution = data.restitution;
		entity.linearDampingFactor = data.linearDampingFactor;
		entity.angularDampingFactor = data.angularDampingFactor;
		return entity;
	}

	public velocityX: number = 0;
	public velocityY: number = 0;
	public angularVelocity: number = 0;
	public isStatic: boolean = false;
	public minX: number = 0;
	public minY: number = 0;
	public maxX: number = 0;
	public maxY: number = 0;
	public frictionCoefficient: number = 0;
	public restitution: number = 1;
	public linearDampingFactor: number = 1;
	public angularDampingFactor: number = 1;
	public points: number[];
	public isCircle: boolean;
	public readonly index: number = Entity.entityIndexTicker++;
	public _angle: number = 0;
	public _mass: number = 1;
	public _inertiaDirty: boolean = true;
	public _inertia: number = 1;
	public _positionX: number;
	public _positionY: number;
	public _radius: number;
	public constructor(positionX: number, positionY: number, radius: number, points?: number[], scaleToEntity: boolean = true) {
		if (points === undefined) {
			this.points = [];
			this.isCircle = true;
			this._positionX = positionX;
			this._positionY = positionY;
			this._radius = radius;
			this.minX = this._positionX - radius;
			this.minY = this._positionY - radius;
			this.maxX = this._positionX + radius;
			this.maxY = this._positionY + radius;
		} else {
			this.isCircle = false;
			if (scaleToEntity) {
				this.points = [];
				this._positionX = positionX;
				this._positionY = positionY;
				this._radius = radius;
				const circle: Circle = computeMEC(points.slice());
				const factor: number = radius / circle.radius;
				for (let i: number = 0; i < points.length; i += 2) {
					this.points.push(positionX + (points[i] - circle.x) * factor, positionY + (points[i + 1] - circle.y) * factor);
				}
				this.updateBox();
			} else {
				this.points = points.slice();
				const circle = computeMEC(points.slice());
				this._positionX = circle.x;
				this._positionY = circle.y;
				this._radius = circle.radius;
				this.updateBox();
				if (positionX !== 0) {
					this.positionX += positionX;
				}
				if (positionY !== 0) {
					this.positionY += positionY;
				}
				if (radius !== 1) {
					this.radius += radius;
				}
			}
		}
	}

	public get positionX(): number {
		return this._positionX;
	}

	public set positionX(x: number) {
		const distance: number = x - this._positionX;
		this._positionX = x;
		this.minX += distance;
		this.maxX += distance;
		if (this.isCircle) {
			return;
		}
		for (let i: number = 0; i < this.points.length; i += 2) {
			this.points[i] += distance;
		}
	}

	public get positionY(): number {
		return this._positionY;
	}

	public set positionY(y: number) {
		const distance: number = y - this._positionY;
		this._positionY = y;
		this.minY += distance;
		this.maxY += distance;
		if (this.isCircle) {
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
		if (this.isCircle) {
			return;
		}
		const cos: number = Math.cos(delta);
		const sin: number = Math.sin(delta);
		for (let i = 0; i < this.points.length; i += 2) {
			const relativeX: number = this.points[i] - this._positionX;
			const relativeY: number = this.points[i + 1] - this._positionY;
			this.points[i] = this._positionX + relativeX * cos - relativeY * sin;
			this.points[i + 1] = this._positionY + relativeX * sin + relativeY * cos;
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
		if (this.isCircle) {
			this._radius = radius;
			this.minX = this._positionX - radius;
			this.minY = this._positionY - radius;
			this.maxX = this._positionX + radius;
			this.maxY = this._positionY + radius;
			return;
		}
		const factor = radius / this._radius;
		this._radius = radius;
		for (let i: number = 0; i < this.points.length; i += 2) {
			this.points[i] = this._positionX + (this.points[i] - this._positionX) * factor;
			this.points[i + 1] = this._positionY + (this.points[i + 1] - this._positionY) * factor;
		}
		this.updateBox();
	}

	public get inertia(): number {
		if (this._inertiaDirty) {
			this.updateInertia();
		}
		return this._inertia;
	}

	public updateBox(): void {
		if (this.isCircle) {
			this.minX = this._positionX - this._radius;
			this.minY = this._positionY - this._radius;
			this.maxX = this._positionX + this._radius;
			this.maxY = this._positionY + this._radius;
		} else {
			this.minX = this.points[0];
			this.minY = this.points[1];
			this.maxX = this.points[0];
			this.maxY = this.points[1];
			for (let i: number = 2; i < this.points.length; i += 2) {
				const pointX: number = this.points[i];
				const pointY: number = this.points[i + 1];
				if (this.minX > pointX) {
					this.minX = pointX;
				}
				if (this.minY > pointY) {
					this.minY = pointY;
				}
				if (this.maxX < pointX) {
					this.maxX = pointX;
				}
				if (this.maxY < pointY) {
					this.maxY = pointY;
				}
			}
		}
	}

	public updateInertia(): void {
		if (this.isCircle) {
			this._inertia = 0.5 * this._mass * this._radius * this._radius;
			return;
		}
		let area2Sum: number = 0;
		let momentumSum: number = 0;
		for (let i: number = 0, j: number = this.points.length - 2; i < this.points.length; j = i, i += 2) {
			const currentPointX: number = this.points[i] - this._positionX;
			const currentPointY: number = this.points[i + 1] - this._positionY;
			const previousPointX: number = this.points[j] - this._positionX;
			const previousPointY: number = this.points[j + 1] - this._positionY;
			const cross: number = currentPointX * previousPointY - previousPointX * currentPointY;
			area2Sum += cross;
			momentumSum += cross * (currentPointX * currentPointX + currentPointX * previousPointX + previousPointX * previousPointX + currentPointY * currentPointY + currentPointY * previousPointY + previousPointY * previousPointY);
		}
		this._inertia = this._mass * momentumSum / 6 / area2Sum;
		this._inertiaDirty = false;
	}

	public moveBy(distanceX: number, distanceY: number): void {
		this._positionX += distanceX;
		this._positionY += distanceY;
		this.minX += distanceX;
		this.maxX += distanceX;
		this.minY += distanceY;
		this.maxY += distanceY;
		if (this.isCircle) {
			return;
		}
		for (let i = 0; i < this.points.length; i += 2) {
			this.points[i] += distanceX;
			this.points[i + 1] += distanceY;
		}
	}

	public applyImpulse(impulseX: number, impulseY: number, contactX: number, contactY: number): void {
		if (this.isStatic) {
			return;
		}
		const inverseMass: number = 1 / this.mass;
		this.velocityX += impulseX * inverseMass;
		this.velocityY += impulseY * inverseMass;
		this.angularVelocity += ((contactX - this._positionX) * impulseY - (contactY - this._positionY) * impulseX) * (1 / this.inertia);
	}

	public clearForces(): void {
		this.velocityX = 0;
		this.velocityY = 0;
		this.angularVelocity = 0;
	}

	public isPointInside(pointX: number, pointY: number): boolean {
		if (this.isCircle) {
			const distanceX = pointX - this._positionX;
			const distanceY = pointY - this._positionY;
			return (distanceX * distanceX + distanceY * distanceY) <= (this._radius * this._radius);
		}
		return isPointInPolygon(pointX, pointY, this.points);
	}

	public clone(): Entity {
		const cloneEntity: Entity = new Entity(this._positionX, this._positionY, this._radius);
		cloneEntity.velocityX = this.velocityX;
		cloneEntity.velocityY = this.velocityY;
		cloneEntity.angle = this.angle;
		cloneEntity.angularVelocity = this.angularVelocity;
		cloneEntity.isStatic = this.isStatic;
		cloneEntity.frictionCoefficient = this.frictionCoefficient;
		cloneEntity.restitution = this.restitution;
		cloneEntity.linearDampingFactor = this.linearDampingFactor;
		cloneEntity.angularDampingFactor = this.angularDampingFactor;
		cloneEntity._mass = this._mass;
		cloneEntity._inertiaDirty = this._inertiaDirty;
		cloneEntity._inertia = this._inertia;
		if (this.isCircle) {
			return cloneEntity;
		}
		cloneEntity.points = this.points.slice();
		cloneEntity.minX = this.minX;
		cloneEntity.minY = this.minY;
		cloneEntity.maxX = this.maxX;
		cloneEntity.maxY = this.maxY;
		return cloneEntity;
	}

	public earthlyShackles(minX: number, minY: number, maxX: number, maxY: number): void {
		if (this.positionX < minX) {
			this.positionX = minX;
			this.velocityX *= -this.restitution;
		} else if (this.positionX > maxX) {
			this.positionX = maxX;
			this.velocityX *= -this.restitution;
		}
		if (this.positionY < minY) {
			this.positionY = minY;
			this.velocityY *= -this.restitution;
		} else if (this.positionY > maxY) {
			this.positionY = maxY;
			this.velocityY *= -this.restitution;
		}
	}

	public update(): void {
		if (Math.abs(this.velocityX) > 1e-3) {
			this.positionX += this.velocityX *= this.linearDampingFactor;
		}
		if (Math.abs(this.velocityY) > 1e-3) {
			this.positionY += this.velocityY *= this.linearDampingFactor;
		}
		if (Math.abs(this.angularVelocity) > 1e-5) {
			this.angle += this.angularVelocity *= this.angularDampingFactor;
		}
	}
}
