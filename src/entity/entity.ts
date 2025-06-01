import { Box } from "../geometry/box";
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
	public box: Box;
	public angle: number;
	public angularVelocity: number;
	constructor(x: number, y: number, radius: number) {
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
		this.box = {
			minX: this.position.x - radius,
			minY: this.position.y - radius,
			maxX: this.position.x + radius,
			maxY: this.position.y + radius
		};
		this.angle = 0;
		this.angularVelocity = 0;
	}

	public moveBy(x: number, y: number): void {
		this.position.x += x;
		this.position.y += y;
		this.box.minX += x;
		this.box.minY += y;
		this.box.maxX += x;
		this.box.maxY += y;
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
	}

	public tick(): void {
		this.moveBy(this.velocity.x, this.velocity.y);
		this.turnBy(this.angularVelocity);
	}
}