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
	private static indexTicker: number = 1;
	public index: number;
	public position: Vector2;
	public velocity: Vector2;
	public radius: number;
	public friction: number;
	public mass: number;
	public isStatic: boolean;
	public box: Box;
	constructor(x: number, y: number, radius: number) {
		this.index = Entity.indexTicker++;
		this.position = new Vector2(x, y);
		this.velocity = new Vector2(0, 0);
		this.radius = radius;
		this.friction = 0.95;
		this.mass = 1;
		this.isStatic = false;
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

	public scaleBy(factor: number): void {
		this.radius *= factor;
		this.box.minX = this.position.x - this.radius;
		this.box.minY = this.position.y - this.radius;
		this.box.maxX = this.position.x + this.radius;
		this.box.maxY = this.position.y + this.radius;
	}

	public scaleTo(radius: number): void {
		this.radius = radius;
		this.box.minX = this.position.x - this.radius;
		this.box.minY = this.position.y - this.radius;
		this.box.maxX = this.position.x + this.radius;
		this.box.maxY = this.position.y + this.radius;
	}
}

export class Collision {
	public static collide(instance: Entity, other: Entity): void {
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared === 0 || distanceSquared >= minimumDistance * minimumDistance) {
			return;
		}
		if (instance.isStatic && other.isStatic) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		const inverseDistance: number = 1 / distance;
		const normalX: number = distanceX * inverseDistance;
		const normalY: number = distanceY * inverseDistance;
		const overlap: number = minimumDistance - distance;
		if (instance.isStatic) {
			other.moveBy(normalX * overlap, normalY * overlap);
		} else if (other.isStatic) {
			instance.moveBy(-normalX * overlap, -normalY * overlap);
		} else {
			const totalMass: number = instance.mass + other.mass;
			if (totalMass > 0) {
				const invTotalMass: number = 1 / totalMass;
				const instanceCorrectionFraction: number = other.mass * invTotalMass;
				const otherCorrectionFraction: number = 1 - instanceCorrectionFraction;
				instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
				other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
			} else {
				const halfOverlap: number = overlap * 0.5;
				instance.moveBy(-normalX * halfOverlap, -normalY * halfOverlap);
				other.moveBy(normalX * halfOverlap, normalY * halfOverlap);
			}
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : 1 / instance.mass;
		const otherInverseMass: number = other.isStatic ? 0 : 1 / other.mass;
		if (instanceInverseMass === 0 && otherInverseMass === 0) {
			return;
		}
		const impulseDenominator: number = instanceInverseMass + otherInverseMass;
		const invImpulseDenominator: number = 1 / impulseDenominator;
		const impulseMagnitude: number = -2 * velocityAlongNormal * invImpulseDenominator;
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.velocity.x -= impulseX * instanceInverseMass;
			instance.velocity.y -= impulseY * instanceInverseMass;
		}
		if (!other.isStatic) {
			other.velocity.x += impulseX * otherInverseMass;
			other.velocity.y += impulseY * otherInverseMass;
		}
	}
}

interface QuadTreeChildren {
	topLeft: QuadTree;
	topRight: QuadTree;
	bottomLeft: QuadTree;
	bottomRight: QuadTree;
}

class QuadTree {
	public static maxEntities: number = 8;
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

	public query(box: Box, result: Set<Entity> = new Set<Entity>()): Set<Entity> {
		if (this.children === null) {
			for (let i: number = 0; i < this.entities.length; i++) {
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
		return result;
	}

	public clear(): void {
		this.entities = [];
		this.children = null;
	}
}

function getPairIndex(index1: number, index2: number): number {
	let minimumIndex: number;
	let maximumIndex: number;
	if (index1 < index2) {
		minimumIndex = index1;
		maximumIndex = index2;
	} else {
		minimumIndex = index2;
		maximumIndex = index1;
	}
	return maximumIndex * maximumIndex + minimumIndex;
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
		const index: number = this.entities.indexOf(entity);
		if (index === -1) {
			return false;
		}
		this.entities.splice(index, 1);
		return true;
	}

	public destroy() {

	}

	public tick(): void {
		this.quadTree.clear();
		const length: number = this.entities.length;
		for (let i: number = 0; i < length; i++) {
			const instance: Entity = this.entities[i];
			instance.tick();
			this.quadTree.insert(instance);
		}
		const processedCollisions: Set<number> = new Set<number>();
		for (let i: number = 0; i < length; i++) {
			const instance: Entity = this.entities[i];
			const potentialColliders: Set<Entity> = this.quadTree.query(instance.box);
			for (const other of potentialColliders) {
				if (instance.index === other.index) {
					continue;
				}
				const pairIndex: number = getPairIndex(instance.index, other.index);
				if (processedCollisions.has(pairIndex)) {
					continue;
				}
				processedCollisions.add(pairIndex);
				if (boxesIntersect(instance.box, other.box)) {
					Collision.collide(instance, other);
				}
			}
		}

	}
}