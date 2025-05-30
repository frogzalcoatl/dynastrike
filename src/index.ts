export interface Box {
	minX: number;
	minY: number;
	maxX: number;
	maxY: number;
}

export function boxesIntersect(instance: Box, other: Box): boolean {
	return !(instance.minX >= other.maxX || instance.maxX <= other.minX || instance.minY >= other.maxY || instance.maxY <= other.minY);
}

export class Vector2 {
	public x: number;
	public y: number;
	constructor(x: number, y: number) {
		this.x = x;
		this.y = y;
	}
}

export class Entity {
	public position: Vector2;
	public velocity: Vector2;
	public radius: number;
	public friction: number;
	public box: Box;
	constructor(x: number, y: number, radius: number) {
		this.position = new Vector2(x, y);
		this.velocity = new Vector2(0, 0);
		this.radius = radius;
		this.friction = 1;
		this.box = {
			minX: this.position.x - radius,
			minY: this.position.y - radius,
			maxX: this.position.x + radius,
			maxY: this.position.y + radius
		};
	}

	public moveTo(x: number, y: number): void {
		this.position.x = x;
		this.position.y = y;
		this.box.minX = x - this.radius;
		this.box.minY = y - this.radius;
		this.box.maxX = x + this.radius;
		this.box.maxY = y + this.radius;
	}

	public moveBy(x: number, y: number): void {
		this.position.x += x;
		this.position.y += y;
		this.box.minX += x;
		this.box.minY += y;
		this.box.maxX += x;
		this.box.maxY += y;
	}

	public tick(): void {
		this.velocity.x *= this.friction;
		this.velocity.y *= this.friction;
		this.moveBy(this.velocity.x, this.velocity.y);
	}
}

export class Collision {
	public static collide(instance: Entity, other: Entity): void {
		const distanceX = other.position.x - instance.position.x;
		const distanceY = other.position.y - instance.position.y;
		const distanceSquared = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance) {
			return;
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
			return;
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

interface QuadTreeChildren {
	topLeft: QuadTree;
	topRight: QuadTree;
	bottomLeft: QuadTree;
	bottomRight: QuadTree;
}

class QuadTree {
	public static maxEntities = 8;
	public static maxLevels = 5;
	public box: Box;
	public level: number;
	public children: QuadTreeChildren | null;
	public entities: Entity[];
	constructor(box: Box, level: number) {
		this.box = box;
		this.level = level;
		this.children = null;
		this.entities = [];
	}

	public split(): void {
		const halfWidth: number = (this.box.maxX - this.box.minX) / 2;
		const halfHeight: number = (this.box.maxY - this.box.minY) / 2;
		const middleX: number = this.box.minX + halfWidth;
		const middleY: number = this.box.minY + halfHeight;
		const level: number = this.level - 1;
		this.children = {
			topLeft: new QuadTree({ minX: this.box.minX, minY: this.box.minY, maxX: middleX, maxY: middleY }, level),
			topRight: new QuadTree({ minX: middleX, minY: this.box.minY, maxX: this.box.maxX, maxY: middleY }, level),
			bottomLeft: new QuadTree({ minX: this.box.minX, minY: middleY, maxX: middleX, maxY: this.box.maxY }, level),
			bottomRight: new QuadTree({ minX: middleX, minY: middleY, maxX: this.box.maxX, maxY: this.box.maxY }, level)
		};
		for (let i: number = 0; i < this.entities.length; i++) {
			this.insert(this.entities[i]);
		}
		this.entities = [];
	}

	public insert(entity: Entity): void {
		if (this.children === null) {
			this.entities.push(entity);
			if (this.level > 0 && this.entities.length > 8) {
				this.split();
			}
			return;
		}
		if (boxesIntersect(this.children.topLeft.box, entity.box)) {
			this.children.topLeft.insert(entity);
		}
		if (boxesIntersect(this.children.topRight.box, entity.box)) {
			this.children.topRight.insert(entity);
		}
		if (boxesIntersect(this.children.bottomLeft.box, entity.box)) {
			this.children.bottomLeft.insert(entity);
		}
		if (boxesIntersect(this.children.bottomRight.box, entity.box)) {
			this.children.bottomRight.insert(entity);
		}
	}

	public query(box: Box, result: Set<Entity> = new Set()): Entity[] {
		if (this.children === null) {
			for (let i = 0; i < this.entities.length; i++) {
				const entity: Entity = this.entities[i];
				if (boxesIntersect(entity.box, box)) {
					result.add(entity);
				}
			}
		} else {
			if (boxesIntersect(this.children.topLeft.box, box)) {
				this.children.topLeft.query(box, result);
			}
			if (boxesIntersect(this.children.topRight.box, box)) {
				this.children.topRight.query(box, result);
			}
			if (boxesIntersect(this.children.bottomLeft.box, box)) {
				this.children.bottomLeft.query(box, result);
			}
			if (boxesIntersect(this.children.bottomRight.box, box)) {
				this.children.bottomRight.query(box, result);
			}
		}
		const output: Entity[] = [];
		for (const entity of result) {
			output.push(entity);
		}
		return output;
	}

	public collisions(): void {
		if (this.children === null) {
			const entities: Entity[] = this.entities.slice().sort((a, b) => a.box.minX - b.box.minX);
			for (let i = 0; i < entities.length; i++) {
				const instance: Entity = entities[i];
				for (let j = i + 1; j < entities.length; j++) {
					const other: Entity = entities[j];
					if (other.box.minX > instance.box.maxX) {
						break;
					}
					if (boxesIntersect(instance.box, other.box)) {
						Collision.collide(instance, other);
					}
				}
			}
		} else {
			this.children.topLeft.collisions();
			this.children.topRight.collisions();
			this.children.bottomLeft.collisions();
			this.children.bottomRight.collisions();
		}
	}

	public clear(): void {
		this.entities = [];
		this.children = null;
	}
}

export class Scene {
	public entities: Entity[];
	public quadTree: QuadTree;
	constructor(box: Box) {
		this.entities = [];
		this.quadTree = new QuadTree(box, 5);
	}

	public addEntity(entity: Entity): number {
		if (this.entities.includes(entity)) {
			return -1;
		}
		return this.entities.push(entity);
	}

	public removeEntity(entity: Entity): boolean {
		const index = this.entities.indexOf(entity);
		if (index === -1) {
			return false;
		}
		this.entities.splice(index, 1);
		return true;
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
		this.quadTree.clear();
		const length: number = this.entities.length;
		for (let i: number = 0; i < length; i++) {
			const instance: Entity = this.entities[i];
			instance.tick();
			this.quadTree.insert(instance);
		}
		this.quadTree.collisions();
	}
}