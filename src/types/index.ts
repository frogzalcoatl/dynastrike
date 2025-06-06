import { QuadTree } from "../spatial/quadtree";

export interface Vector2 {
	x: number;
	y: number;
}

export interface ProjectionRange {
	minimum: number;
	maximum: number;
}

export type BoundaryBuffer = number[];

export type Triangle = [number, number, number, number, number, number];

export interface Circle {
	x: number;
	y: number;
	radius: number;
}

export interface Box {
	minX: number;
	minY: number;
	maxX: number;
	maxY: number;
}

export interface SATResult {
	penetration: number;
	normalX: number;
	normalY: number;
	collided: boolean;
}

export interface ContactResult {
	penetration: number;
	normalX: number;
	normalY: number;
}

export interface Projection {
	x: number;
	y: number;
	distanceSquared: number;
}

export interface QuadTreeChildren {
	topLeft: QuadTree;
	topRight: QuadTree;
	bottomLeft: QuadTree;
	bottomRight: QuadTree;
}