export class Vector2 {
	public x: number;
	public y: number;
	constructor(x: number, y: number) {
		this.x = x;
		this.y = y;
	}
}

export interface AABB {
	minX: number;
	minY: number;
	maxX: number;
	maxY: number;
}

export class Entity {
	public position: Vector2;
	public velocity: Vector2;
	public radius: number;
	public friction: number;
	public aabb: AABB;
	constructor(x: number, y: number, radius: number) {
		this.position = new Vector2(x, y);
		this.velocity = new Vector2(0, 0);
		this.radius = radius;
		this.friction = 0.95;
		this.aabb = {
			minX: this.position.x - radius,
			minY: this.position.y - radius,
			maxX: this.position.x + radius,
			maxY: this.position.y + radius
		};
	}

	public moveTo(x: number, y: number): void {
		this.position.x = x;
		this.position.y = y;
		this.aabb.minX = x - this.radius;
		this.aabb.minY = y - this.radius;
		this.aabb.maxX = x + this.radius;
		this.aabb.maxY = y + this.radius;
	}

	public moveBy(x: number, y: number): void {
		this.position.x += x;
		this.position.y += y;
		this.aabb.minX += x;
		this.aabb.minY += y;
		this.aabb.maxX += x;
		this.aabb.maxY += y;
	}

	public tick(deltaTime: number): void {
		const decay = Math.pow(this.friction, deltaTime);
		this.velocity.x *= decay;
		this.velocity.y *= decay;
		this.moveBy(this.velocity.x * deltaTime, this.velocity.y * deltaTime);
	}
}

export class Scene {
	public entities: Set<Entity>;
	public lastTickTime: number;
	public intervalIndex: number;
	constructor(tickrate: number) {
		this.entities = new Set();
		if (tickrate === 0) {
			this.lastTickTime = -1;
			this.intervalIndex = -1;
		} else {
			this.lastTickTime = performance.now();
			this.intervalIndex = setInterval(() => this.tick(), tickrate);
		}
	}

	public addEntity(entity: Entity) {
		if (this.entities.has(entity)) return false;
		this.entities.add(entity);
		return true;
	}

	public removeEntity(entity: Entity) {
		return this.entities.delete(entity);
	}

	public destroy() {
		if (this.lastTickTime !== -1) {
			clearInterval(this.intervalIndex);
		}
	}

	public tick(): void {
		let deltaTime = 1;
		if (this.lastTickTime !== -1) {
			const currentTime = performance.now();
			deltaTime = (currentTime - this.lastTickTime) / 10;
			this.lastTickTime = currentTime;
		}
		for (const entity of this.entities.values()) {
			entity.tick(deltaTime);
		}
	}
}