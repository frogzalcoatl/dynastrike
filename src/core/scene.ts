import { Collision } from "../collision/collide";
import { QuadTree } from "../spatial/quadtree";
import { Box } from "../types";
import { Entity } from "./entity";

export function getPairIndex(index1: number, index2: number): number {
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
	public entities: Set<Entity>;
	public quadTree: QuadTree;
	public box: Box;
	constructor(box: Box) {
		this.entities = new Set<Entity>();
		this.quadTree = new QuadTree(box, 32);
		this.box = box;
	}

	public addEntity(entity: Entity): void {
		this.entities.add(entity);
	}

	public removeEntity(entity: Entity): void {
		this.entities.delete(entity);
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
		this.step(1);
	}

	private step(deltaTimeSeconds: number): void {
		this.quadTree.clear();
		const processedCollisions: Set<number> = new Set<number>();
		for (const instance of this.entities.values()) {
			instance.update(deltaTimeSeconds);
			const potentialColliders: Set<Entity> = this.quadTree.query(instance.box);
			this.quadTree.insertEntity(instance);
			for (const other of potentialColliders.values()) {
				const pairIndex: number = getPairIndex(instance.index, other.index);
				if (processedCollisions.has(pairIndex)) {
					continue;
				}
				processedCollisions.add(pairIndex);
				if (instance.isStatic && other.isStatic) {
					continue;
				}
				Collision.collide(instance, other);
				this.earthlyShackles(other);
			}
			this.earthlyShackles(instance);
		}
	}
}