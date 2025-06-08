import { Collision } from "../physics/collide";
import { QuadTree } from "../spatial/quadtree";
import { Entity } from "./entity";

export class Scene {
	public entities: Set<Entity> = new Set<Entity>();
	public minX: number;
	public minY: number;
	public maxX: number;
	public maxY: number;
	public grid: QuadTree;
	constructor(minX: number, minY: number, maxX: number, maxY: number, topLevel: number) {
		this.minX = minX;
		this.minY = minY;
		this.maxX = maxX;
		this.maxY = maxY;
		this.grid = new QuadTree(minX, minY, maxX, maxY, topLevel);
	}

	public addEntity(entity: Entity): void {
		this.entities.add(entity);
	}

	public removeEntity(entity: Entity): void {
		this.entities.delete(entity);
	}

	public query(minX: number, minY: number, maxX: number, maxY: number): Set<Entity> {
		return this.grid.query(minX, minY, maxX, maxY);
	}

	public update(): void {
		this.grid.clear();
		const processedCollisions: Set<number> = new Set<number>();
		for (const instance of this.entities.values()) {
			instance.update();
			const potentialColliders: SetIterator<Entity> = this.grid.query(instance.minX, instance.minY, instance.maxX, instance.maxY).values();
			this.grid.insert(instance);
			for (const other of potentialColliders) {
				const pairIndex: number = this.getPairIndex(instance.index, other.index);
				if (processedCollisions.has(pairIndex) || (instance.isStatic && other.isStatic)) {
					continue;
				}
				processedCollisions.add(pairIndex);
				Collision.collide(instance, other);
				other.earthlyShackles(this.minX, this.minY, this.maxX, this.maxY);
			}
			instance.earthlyShackles(this.minX, this.minY, this.maxX, this.maxY);
		}
	}

	private getPairIndex(instanceIndex: number, otherIndex: number): number {
		return instanceIndex < otherIndex ? otherIndex * otherIndex + instanceIndex : instanceIndex * instanceIndex + otherIndex;
	}
}