import { Collision } from "../physics/collide";
import { QuadTree } from "../spatial/quadtree";
import { Box } from "../types";
import { Entity } from "./entity";

export class Scene {
	public box: Box;
	public grid: QuadTree;
	public entities: Set<Entity> = new Set<Entity>();
	constructor(box: Box, topLevel: number) {
		this.box = box;
		this.grid = new QuadTree(this.box, topLevel);
	}

	public addEntity(entity: Entity): void {
		this.entities.add(entity);
	}

	public removeEntity(entity: Entity): void {
		this.entities.delete(entity);
	}

	public query(box: Box): Set<Entity> {
		return this.grid.query(box);
	}

	public update() {
		this.step(1);
	}

	public step(deltaTime: number): void {
		this.grid.clear();
		const processedCollisions: Set<number> = new Set<number>();
		for (const instance of this.entities.values()) {
			instance.update(deltaTime);
			const potentialColliders: SetIterator<Entity> = this.grid.query(instance.box).values();
			this.grid.insert(instance);
			for (const other of potentialColliders) {
				const pairIndex: number = this.getPairIndex(instance.index, other.index);
				if (processedCollisions.has(pairIndex) || (instance.isStatic && other.isStatic)) {
					continue;
				}
				processedCollisions.add(pairIndex);
				Collision.collide(instance, other);
				other.earthlyShackles(this.box);
			}
			instance.earthlyShackles(this.box);
		}
	}

	private getPairIndex(instanceIndex: number, otherIndex: number): number {
		return instanceIndex < otherIndex ? otherIndex * otherIndex + instanceIndex : instanceIndex * instanceIndex + otherIndex;
	}
}