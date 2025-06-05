import { Entity } from "../entity/entity";
import { Box } from "../geometry/box";
import { Collision } from "../physics/collision";
import { QuadTree } from "../physics/quadtree";

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
	public box: Box;
	constructor(box: Box) {
		this.entities = [];
		this.quadTree = new QuadTree(box, 8);
		this.box = box;
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

	private earthlyShackles(entity: Entity): void {
		if (entity.positionX < this.box.minX) {
			entity.positionX = this.box.minX;
			entity.positionalVelocity.x = 0;
		} else if (entity.positionX > this.box.maxX) {
			entity.positionX = this.box.maxX;
			entity.positionalVelocity.x = 0;
		}
		if (entity.positionY < this.box.minY) {
			entity.positionY = this.box.minY;
			entity.positionalVelocity.y = 0;
		} else if (entity.positionY > this.box.maxY) {
			entity.positionY = this.box.maxY;
			entity.positionalVelocity.y = 0;
		}
	}

	public update(): void {
		this.quadTree.clear();
		const processedCollisions: Set<number> = new Set<number>();
		const length: number = this.entities.length;
		for (let i: number = 0; i < length; i++) {
			const instance: Entity = this.entities[i];
			instance.update();
			const potentialColliders: Set<Entity> = this.quadTree.query(instance.box);
			this.quadTree.insert(instance);
			for (const other of potentialColliders) {
				const pairIndex: number = getPairIndex(instance.index, other.index);
				if (processedCollisions.has(pairIndex)) {
					continue;
				}
				processedCollisions.add(pairIndex);
				Collision.collide(instance, other);
				this.earthlyShackles(other);
			}
			this.earthlyShackles(instance);
		}
	}
}