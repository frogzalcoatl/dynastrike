import { Entity } from "../core/entity";
import { boxesIntersect } from "../geometry/box";
import { Box, QuadTreeChildren } from "../types";

export class QuadTree {
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
		for (let i: number = 0, length = this.entities.length; i < length; i++) {
			this.insert(this.entities[i]);
		}
		this.entities = [];
	}

	public insert(entity: Entity): void {
		if (this.children === null) {
			this.entities.push(entity);
			if (this.entities.length > 11) {
				this.split();
			}
		} else {
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