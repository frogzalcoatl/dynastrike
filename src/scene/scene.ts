import { Entity } from "../entity/entity";
import { Box, boxesIntersect } from "../geometry/box";
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
	constructor(box: Box) {
		this.entities = [];
		this.quadTree = new QuadTree(box, 7);
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