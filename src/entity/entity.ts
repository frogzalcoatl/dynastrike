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
	public mass: number;
	public isStatic: boolean;
	public angle: number;
	public angularVelocity: number;
	public points: number[] | null;
	public box: Box;
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
		this.mass = 1;
		this.isStatic = false;
		this.angle = 0;
		this.angularVelocity = 0;
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
			for (let i = 0; i < points.length; i += 2) {
				this.points.push(this.position.x + (points[i] - circle.x) * factor, this.position.y + (points[i + 1] - circle.y) * factor);
			}
			this.box = computeBox(this.points);
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
		this.box.minX = this.position.x - this.radius;
		this.box.minY = this.position.y - this.radius;
		this.box.maxX = this.position.x + this.radius;
		this.box.maxY = this.position.y + this.radius;
	}

	public turnBy(radians: number): void {
		this.angle = (((this.angle + radians) % TAU) + TAU) % TAU;
		if (this.points !== null) {
			const cos = Math.cos(radians);
			const sin = Math.sin(radians);
			for (let i = 0; i < this.points.length; i += 2) {
				const relativeX = this.points[i] - this.position.x;
				const relativeY = this.points[i + 1] - this.position.y;
				this.points[i] = this.position.x + relativeX * cos - relativeY * sin;
				this.points[i + 1] = this.position.y + relativeX * sin + relativeY * cos;
			}
			this.updateBox();
		}
	}

	public tick(): void {
		this.moveBy(this.velocity.x *= 0.9, this.velocity.y *= 0.9);
		this.turnBy(this.angularVelocity);
	}
}