import { Collision } from "../physics/collide";
import { BaseJoint } from "../physics/joint";
import { QuadTree } from "../spatial/quadtree";
import { Entity } from "./entity";

export class Scene {
	public entities: Set<Entity> = new Set<Entity>();
	public joints: BaseJoint[] = [];
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

	public addJoint(joint: BaseJoint): void {
		this.joints.push(joint);
	}

	public removeJoint(joint: BaseJoint): void {
		const index: number = this.joints.indexOf(joint);
		if (index === -1) {
			return;
		}
		const lastIndex: number = this.joints.length - 1;
		const temporaryJoint: BaseJoint = this.joints[index];
		this.joints[index] = this.joints[lastIndex];
		this.joints[lastIndex] = temporaryJoint;
		this.joints.pop();
	}

	public query(minX: number, minY: number, maxX: number, maxY: number): Set<Entity> {
		return this.grid.query(minX, minY, maxX, maxY);
	}

	public update(): void {
		this.grid.clear();
		const processedCollisions: Set<number> = new Set<number>();
		for (const instance of this.entities.values()) {
			const instanceIndex: number = instance.index;
			instance.update();
			const potentialColliders: Set<Entity> = this.grid.query(instance.minX, instance.minY, instance.maxX, instance.maxY);
			this.grid.insert(instance);
			if (potentialColliders.size !== 0) {
				for (const other of potentialColliders.values()) {
					const otherIndex: number = other.index;
					const pairIndex: number = instanceIndex < otherIndex ? (otherIndex << 16) | instanceIndex : (instanceIndex << 16) | otherIndex;
					if (processedCollisions.has(pairIndex)) {
						continue;
					}
					processedCollisions.add(pairIndex);
					if (instance.isStatic && other.isStatic) {
						continue;
					}
					Collision.collide(instance, other);
					if (!other.isStatic) {
						other.earthlyShackles(this.minX, this.minY, this.maxX, this.maxY);
					}
				}
			}
			if (!instance.isStatic) {
				instance.earthlyShackles(this.minX, this.minY, this.maxX, this.maxY);
			}
		}
		for (let i: number = this.joints.length - 1; i >= 0; i--) {
			const joint: BaseJoint = this.joints[i];
			joint.update();
			if (joint.broken) {
				this.joints[i] = this.joints[this.joints.length - 1];
				this.joints.pop();
			}
		}
	}
}