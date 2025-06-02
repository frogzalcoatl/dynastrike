import { Box, computeBox } from "../geometry/box";
import { Circle, computeMEC } from "../geometry/mec";
import { Vector2 } from "../geometry/vector";

const TAU = Math.PI * 2;
let entityIndexTicker: number = 1;
export class Entity {
	public index: number;
	public position: Vector2;
	public velocity: Vector2;
	public radius: number;
	public isStatic: boolean;
	public angle: number;
	public angularVelocity: number;
	public points: number[] | null;
	public box: Box;
	private inertiaDirty: boolean;
	private _inertia: number;
	private _mass: number;
	constructor(x: number, y: number, radius: number, points: number[] | null = null) {
		this.index = entityIndexTicker++;
		this.position = {
			x: x,
			y: y
		};
		this.velocity = {
			x: 0,
			y: 0
		};
		this.radius = radius;
		this.isStatic = false;
		this.angle = 0;
		this.angularVelocity = 0;
		this.inertiaDirty = true;
		this._inertia = 1;
		this._mass = 1;
		if (points === null) {
			this.points = null;
			this.box = {
				minX: this.position.x - radius,
				minY: this.position.y - radius,
				maxX: this.position.x + radius,
				maxY: this.position.y + radius
			};
		} else {
			this.points = [];
			const circle: Circle = computeMEC(points.slice());
			const factor: number = circle.radius < 1e-9 ? 0 : this.radius / circle.radius;
			for (let i: number = 0; i < points.length; i += 2) {
				this.points.push(this.position.x + (points[i] - circle.x) * factor, this.position.y + (points[i + 1] - circle.y) * factor);
			}
			this.box = computeBox(this.points);
		}
	}

	public get inertia(): number {
		if (this.inertiaDirty) {
			this.updateInertia();
			this.inertiaDirty = false;
		}
		return this._inertia;
	}

	public get mass(): number {
		return this._mass;
	}

	public set mass(value: number) {
		this.inertiaDirty = true;
		this._mass = value;
	}

	private updateInertia(): void {
		if (this.points === null) {
			this._inertia = 0.5 * this._mass * this.radius * this.radius;
		} else {
			let area2sum: number = 0;
			let momentumSum: number = 0;
			for (let i: number = 0, j: number = this.points.length - 2; i < this.points.length; j = i, i += 2) {
				const pointX = this.points[i] - this.position.x;
				const pointY = this.points[i + 1] - this.position.y;
				const previousPointX = this.points[j] - this.position.x;
				const previousPointY = this.points[j + 1] - this.position.y;
				const cross = pointX * previousPointY - previousPointX * pointY;
				area2sum += cross;
				momentumSum += cross * (pointX * pointX + pointX * previousPointX + previousPointX * previousPointX + pointY * pointY + pointY * previousPointY + previousPointY * previousPointY);
			}
			this._inertia = (this._mass * momentumSum) / (6 * area2sum);
		}
	}

	public updateBox(): void {
		if (this.points === null) {
			this.box.minX = this.position.x - this.radius;
			this.box.minY = this.position.y - this.radius;
			this.box.maxX = this.position.x + this.radius;
			this.box.maxY = this.position.y + this.radius;
		} else {
			this.box.minX = this.points[0];
			this.box.minY = this.points[1];
			this.box.maxX = this.points[0];
			this.box.maxY = this.points[1];
			for (let i = 2; i < this.points.length; i += 2) {
				const pointX = this.points[i];
				const pointY = this.points[i + 1];
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

	public moveBy(x: number, y: number): void {
		this.position.x += x;
		this.position.y += y;
		this.box.minX += x;
		this.box.minY += y;
		this.box.maxX += x;
		this.box.maxY += y;
		if (this.points !== null) {
			for (let i = 0; i < this.points.length; i += 2) {
				this.points[i] += x;
				this.points[i + 1] += y;
			}
		}
	}

	public scaleBy(factor: number): void {
		this.radius *= factor;
		if (this.points !== null) {
			for (let i: number = 0; i < this.points.length; i += 2) {
				this.points[i] = this.position.x + (this.points[i] - this.position.x) * factor;
				this.points[i + 1] = this.position.y + (this.points[i + 1] - this.position.y) * factor;
			}
			this.updateBox();
		} else {
			this.box.minX = this.position.x - this.radius;
			this.box.minY = this.position.y - this.radius;
			this.box.maxX = this.position.x + this.radius;
			this.box.maxY = this.position.y + this.radius;
		}
		this.inertiaDirty = true;
	}

	public turnBy(radians: number): void {
		this.angle = (((this.angle + radians) % TAU) + TAU) % TAU;
		if (this.points !== null) {
			const cos: number = Math.cos(radians);
			const sin: number = Math.sin(radians);
			for (let i = 0; i < this.points.length; i += 2) {
				const relativeX: number = this.points[i] - this.position.x;
				const relativeY: number = this.points[i + 1] - this.position.y;
				this.points[i] = this.position.x + relativeX * cos - relativeY * sin;
				this.points[i + 1] = this.position.y + relativeX * sin + relativeY * cos;
			}
			this.updateBox();
		}
	}

	public tick(): void {
		const linearFriction: number = 0.92;
		const angularFriction: number = 0.90;
		this.velocity.x *= linearFriction;
		this.velocity.y *= linearFriction;
		this.angularVelocity *= angularFriction;
		this.moveBy(this.velocity.x, this.velocity.y);
		this.turnBy(this.angularVelocity);
	}
}