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
		this.friction = 1;
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

	public tick(): void {
		this.velocity.x *= this.friction;
		this.velocity.y *= this.friction;
		this.moveBy(this.velocity.x, this.velocity.y);
	}
}

export class Scene {
	public entities: Set<Entity>;
	constructor(tickrate: number) {
		this.entities = new Set();
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

	}

	public tick(): void {
		/*let deltaTime = 1;
		if (this.lastTickTime !== -1) {
			const currentTime = performance.now();
			deltaTime = (currentTime - this.lastTickTime) / 10;
			this.lastTickTime = currentTime;
		}*/
		const entities = Array.from(this.entities);
		for (let i = 0; i < entities.length; i++) {
			const instance = entities[i];
			instance.tick();
			for (let j = 0; j < i; j++) {
				const other = entities[j];
				const distanceX = other.position.x - instance.position.x;
				const distanceY = other.position.y - instance.position.y;
				const distanceSquared = distanceX * distanceX + distanceY * distanceY;
				const minimumDistance = instance.radius + other.radius;
				if (distanceSquared >= minimumDistance * minimumDistance) {
					continue;
				}
				const distance = Math.sqrt(distanceSquared) || 1e-4;
				const normalX = distanceX / distance;
				const normalY = distanceY / distance;
				const overlap = minimumDistance - distance;
				const correction = overlap * 0.5;
				instance.moveBy(normalX * -correction, normalY * -correction);
				other.moveBy(normalX * correction, normalY * correction);
				const rVelocityX = other.velocity.x - instance.velocity.x;
				const rVelocityY = other.velocity.y - instance.velocity.y;
				const velocityAlongNormal = rVelocityX * normalX + rVelocityY * normalY;
				if (velocityAlongNormal > 0) {
					continue;
				}
				const impulse = -(1 + 1) * velocityAlongNormal * 0.5;
				const impulseX = impulse * normalX;
				const impulseY = impulse * normalY;
				instance.velocity.x -= impulseX;
				instance.velocity.y -= impulseY;
				other.velocity.x += impulseX;
				other.velocity.y += impulseY;

			}
		}
	}
}