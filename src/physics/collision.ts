import { Entity } from "../entity/entity";
import { Vector2 } from "../geometry/vector";
import { computeAveragePoint } from "./modules/computeAveragePoint";
import { computePenetrationAndNormal, ContactResult } from "./modules/computePenetrationAndNormal";
import { computeSATConvex, SATResult } from "./modules/computeSATConvex";
import { findContactPoint } from "./modules/findContactPoint";
import { isPointInPolygon } from "./modules/isPointInPolygon";
import { Projection, projectPointOnEdge } from "./modules/projectPointOnEdge";
import { Triangle, triangulate } from "./modules/triangle";

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance || (instance.isStatic && other.isStatic)) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		let normalX: number = 0;
		let normalY: number = 0;
		if (distance < 1e-9) {
			const angle: number = Math.PI * 2 * Math.random();
			normalX = Math.cos(angle);
			normalY = Math.sin(angle);
		} else {
			const inverseDistance: number = 1 / distance;
			normalX = distanceX * inverseDistance;
			normalY = distanceY * inverseDistance;
		}
		const overlap: number = minimumDistance - distance;
		if (instance.isStatic) {
			other.moveBy(normalX * overlap, normalY * overlap);
		} else if (other.isStatic) {
			instance.moveBy(-normalX * overlap, -normalY * overlap);
		} else {
			const inverseTotalMass: number = 1 / (instance.mass + other.mass);
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
			other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : (instance.mass === 0 ? 0 : 1 / instance.mass);
		const otherInverseMass: number = other.isStatic ? 0 : (other.mass === 0 ? 0 : 1 / other.mass);
		const impulseMagnitude: number = -2 * velocityAlongNormal / (instanceInverseMass + otherInverseMass);
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

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		if (polygonEntity.points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleRadiusSquared: number = circleEntity.radius * circleEntity.radius;
		const isInside: boolean = isPointInPolygon(circleEntity.position.x, circleEntity.position.y, polygonEntity.points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedContactXTimesPenetration: number = 0;
		let accumulatedContactYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let minDistanceSqForInside: number = Infinity;
		let normalXForInside: number = 0;
		let normalYForInside: number = 0;
		let previousX: number = polygonEntity.points[polygonEntity.points.length - 2];
		let previousY: number = polygonEntity.points[polygonEntity.points.length - 1];
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const currentX: number = polygonEntity.points[i];
			const currentY: number = polygonEntity.points[i + 1];
			const projection: Projection = projectPointOnEdge(circleEntity.position.x, circleEntity.position.y, previousX, previousY, currentX, currentY);
			const contact: ContactResult = computePenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, projection.x, projection.y, circleEntity.radius, polygonEntity.position.x, polygonEntity.position.y, isInside, projection.distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += projection.x * contact.penetration;
				accumulatedContactYTimesPenetration += projection.y * contact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (projection.distanceSquared < minDistanceSqForInside) {
					minDistanceSqForInside = projection.distanceSquared;
					normalXForInside = projection.x - circleEntity.position.x;
					normalYForInside = projection.y - circleEntity.position.y;
				}
			}
			previousX = currentX;
			previousY = currentY;
		}
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const vertexX: number = polygonEntity.points[i];
			const vertexY: number = polygonEntity.points[i + 1];
			const distanceX: number = circleEntity.position.x - vertexX;
			const distanceY: number = circleEntity.position.y - vertexY;
			const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
			const contact: ContactResult = computePenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, vertexX, vertexY, circleEntity.radius, polygonEntity.position.x, polygonEntity.position.y, isInside, distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += vertexX * contact.penetration;
				accumulatedContactYTimesPenetration += vertexY * contact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (distanceSquared < minDistanceSqForInside) {
					minDistanceSqForInside = distanceSquared;
					normalXForInside = vertexX - circleEntity.position.x;
					normalYForInside = vertexY - circleEntity.position.y;
				}
			}
		}
		if (contactCount === 0 && isInside) {
			if (minDistanceSqForInside === Infinity) {
				return;
			}
			const distToBoundary: number = Math.sqrt(minDistanceSqForInside);
			let bestNormalX: number = 0;
			let bestNormalY: number = 0;
			if (distToBoundary < 1e-9) {
				bestNormalX = 0;
				bestNormalY = 1;
			} else {
				const inverseDistance: number = 1 / distToBoundary;
				bestNormalX = normalXForInside * inverseDistance;
				bestNormalY = normalYForInside * inverseDistance;
			}
			const penetration: number = circleEntity.radius + distToBoundary;
			if (polygonEntity.isStatic) {
				circleEntity.moveBy(bestNormalX * penetration, bestNormalY * penetration);
			} else if (circleEntity.isStatic) {
				polygonEntity.moveBy(-bestNormalX * penetration, -bestNormalY * penetration);
			} else {
				const inverseTotalMass = 1 / (polygonEntity.mass + circleEntity.mass);
				const polygonCorrectionFactor = circleEntity.mass * inverseTotalMass;
				const circleCorrectionFactor = polygonEntity.mass * inverseTotalMass;
				polygonEntity.moveBy(-bestNormalX * penetration * polygonCorrectionFactor, -bestNormalY * penetration * polygonCorrectionFactor);
				circleEntity.moveBy(bestNormalX * penetration * circleCorrectionFactor, bestNormalY * penetration * circleCorrectionFactor);
			}
			return;
		}
		if (contactCount === 0) {
			return;
		}
		let weightedNormalX: number = 0;
		let weightedNormalY: number = 0;
		let averageContactX: number = 0;
		let averageContactY: number = 0;
		if (sumPenetration < 1e-9) {
			weightedNormalX = 0;
			weightedNormalY = 0;
			averageContactX = 0;
			averageContactY = 0;
		} else {
			const inverseSum: number = 1 / sumPenetration;
			weightedNormalX = accumulatedNormalXTimesPenetration * inverseSum;
			weightedNormalY = accumulatedNormalYTimesPenetration * inverseSum;
			averageContactX = accumulatedContactXTimesPenetration * inverseSum;
			averageContactY = accumulatedContactYTimesPenetration * inverseSum;
		}
		const averageNormalLength: number = Math.sqrt(weightedNormalX * weightedNormalX + weightedNormalY * weightedNormalY);
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		if (averageNormalLength < 1e-9) {
			finalNormalX = 0;
			finalNormalY = 1;
		} else {
			const inverseLength: number = 1 / averageNormalLength;
			finalNormalX = weightedNormalX * inverseLength;
			finalNormalY = weightedNormalY * inverseLength;
		}
		const correctionMagnitude: number = (sumPenetration / contactCount);
		if (polygonEntity.isStatic) {
			circleEntity.moveBy(finalNormalX * correctionMagnitude, finalNormalY * correctionMagnitude);
		} else if (circleEntity.isStatic) {
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude, -finalNormalY * correctionMagnitude);
		} else {
			const totalMass = polygonEntity.mass + circleEntity.mass;
			const inverseTotalMass = 1 / totalMass;
			const polygonCorrectionFactor = circleEntity.mass * inverseTotalMass;
			const circleCorrectionFactor = polygonEntity.mass * inverseTotalMass;
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude * polygonCorrectionFactor, -finalNormalY * correctionMagnitude * polygonCorrectionFactor);
			circleEntity.moveBy(finalNormalX * correctionMagnitude * circleCorrectionFactor, finalNormalY * correctionMagnitude * circleCorrectionFactor);
		}
		const relativePolygonX: number = averageContactX - polygonEntity.position.x;
		const relativePolygonY: number = averageContactY - polygonEntity.position.y;
		const relativeCircleX: number = averageContactX - circleEntity.position.x;
		const relativeCircleY: number = averageContactY - circleEntity.position.y;
		const relativeVelocityX: number = (circleEntity.velocity.x - circleEntity.angularVelocity * relativeCircleY) - (polygonEntity.velocity.x - polygonEntity.angularVelocity * relativePolygonY);
		const relativeVelocityY: number = (circleEntity.velocity.y + circleEntity.angularVelocity * relativeCircleX) - (polygonEntity.velocity.y + polygonEntity.angularVelocity * relativePolygonX);
		const velocityAlongNormal: number = relativeVelocityX * finalNormalX + relativeVelocityY * finalNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inversePolygonMass: number = polygonEntity.isStatic ? 0 : 1 / polygonEntity.mass;
		const inverseCircleMass: number = circleEntity.isStatic ? 0 : 1 / circleEntity.mass;
		let invInertiaPoly: number = 0;
		if (!polygonEntity.isStatic && polygonEntity.inertia > 1e-9) {
			invInertiaPoly = 1 / polygonEntity.inertia;
		}
		let invInertiaCirc: number = 0;
		if (!circleEntity.isStatic && circleEntity.inertia > 1e-9) {
			invInertiaCirc = 1 / circleEntity.inertia;
		}
		const relativePolygonPerpendicular: number = relativePolygonX * finalNormalY - relativePolygonY * finalNormalX;
		const relativeCirclePerpendicular: number = relativeCircleX * finalNormalY - relativeCircleY * finalNormalX;
		const denominator: number = inversePolygonMass + inverseCircleMass + relativePolygonPerpendicular * relativePolygonPerpendicular * invInertiaPoly + relativeCirclePerpendicular * relativeCirclePerpendicular * invInertiaCirc;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = (-2.0 * velocityAlongNormal) / denominator;
		const impulseX: number = impulseMagnitude * finalNormalX;
		const impulseY: number = impulseMagnitude * finalNormalY;
		if (!polygonEntity.isStatic) {
			polygonEntity.velocity.x -= impulseX * inversePolygonMass;
			polygonEntity.velocity.y -= impulseY * inversePolygonMass;
			polygonEntity.angularVelocity -= (relativePolygonX * impulseY - relativePolygonY * impulseX) * invInertiaPoly;
		}
		if (!circleEntity.isStatic) {
			circleEntity.velocity.x += impulseX * inverseCircleMass;
			circleEntity.velocity.y += impulseY * inverseCircleMass;
			circleEntity.angularVelocity += (relativeCircleX * impulseY - relativeCircleY * impulseX) * invInertiaCirc;
		}
	}

	public static collidePolygonPolygon(instance: Entity, other: Entity): void {
		if (instance.points === null || other.points === null || (instance.isStatic && other.isStatic)) {
			return;
		}
		const instanceTriangles: Triangle[] = triangulate(instance.points);
		const otherTriangles: Triangle[] = triangulate(other.points);
		let penetrationSum: number = 0;
		let accumulatedNormalX: number = 0;
		let accumulatedNormalY: number = 0;
		let accumulatedContactX: number = 0;
		let accumulatedContactY: number = 0;
		let contactCount: number = 0;
		let maxPenetration: number = 0;
		let maxNormalX: number = 0;
		let maxNormalY: number = 0;
		let maxContactX: number = 0;
		let maxContactY: number = 0;
		for (let i: number = 0; i < instanceTriangles.length; ++i) {
			const instanceTriangle = instanceTriangles[i];
			for (let j: number = 0; j < otherTriangles.length; ++j) {
				const otherTriangle = otherTriangles[j];
				const satResult: SATResult = computeSATConvex(instanceTriangle, otherTriangle);
				if (!satResult.collided) {
					continue;
				}
				let normalX: number = satResult.normalX;
				let normalY: number = satResult.normalY;
				const instanceCenter: Vector2 = computeAveragePoint(instanceTriangle);
				const otherCenter: Vector2 = computeAveragePoint(otherTriangle);
				const distanceX: number = otherCenter.x - instanceCenter.x;
				const distanceY: number = otherCenter.y - instanceCenter.y;
				if (normalX * distanceX + normalY * distanceY < 0) {
					normalX = -normalX;
					normalY = -normalY;
				}
				const contactPoint: Vector2 = findContactPoint(instanceTriangle, otherTriangle);
				penetrationSum += satResult.penetration;
				accumulatedNormalX += normalX * satResult.penetration;
				accumulatedNormalY += normalY * satResult.penetration;
				accumulatedContactX += contactPoint.x * satResult.penetration;
				accumulatedContactY += contactPoint.y * satResult.penetration;
				contactCount++;
				if (satResult.penetration > maxPenetration) {
					maxPenetration = satResult.penetration;
					maxNormalX = normalX;
					maxNormalY = normalY;
					maxContactX = contactPoint.x;
					maxContactY = contactPoint.y;
				}
			}
		}
		if (contactCount === 0) {
			return;
		}
		const inversePenetrationSum: number = 1 / penetrationSum;
		const weightedNormalX: number = accumulatedNormalX * inversePenetrationSum;
		const weightedNormalY: number = accumulatedNormalY * inversePenetrationSum;
		const averageContactX: number = accumulatedContactX * inversePenetrationSum;
		const averageContactY: number = accumulatedContactY * inversePenetrationSum;
		const normalLength: number = Math.hypot(weightedNormalX, weightedNormalY);
		let prelimNormalX: number;
		let prelimNormalY: number;
		const normalLengthSignificant: boolean = normalLength > 1e-6;
		if (normalLengthSignificant) {
			const invNormalLength: number = 1.0 / normalLength;
			prelimNormalX = weightedNormalX * invNormalLength;
			prelimNormalY = weightedNormalY * invNormalLength;
		} else {
			const fallbackLength: number = Math.hypot(maxNormalX, maxNormalY);
			if (fallbackLength > 0) {
				const invFallbackLength: number = 1.0 / fallbackLength;
				prelimNormalX = maxNormalX * invFallbackLength;
				prelimNormalY = maxNormalY * invFallbackLength;
			} else {
				const angle: number = Math.PI * 2 * Math.random();
				prelimNormalX = Math.cos(angle);
				prelimNormalY = Math.sin(angle);
			}
		}
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distance: number = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
		let axisX: number = 0;
		let axisY: number = 0;
		if (distance > 1e-9) {
			const invEntDistance: number = 1.0 / distance;
			axisX = distanceX * invEntDistance;
			axisY = distanceY * invEntDistance;
		} else {
			const angle: number = Math.PI * 2 * Math.random();
			axisX = Math.cos(angle);
			axisY = Math.sin(angle);
		}
		const dotAxis: number = prelimNormalX * axisX + prelimNormalY * axisY;
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		let contactX: number = 0;
		let contactY: number = 0;

		if (Math.abs(dotAxis) > 1.0 - 1e-6) {
			const sign: number = (dotAxis < 0) ? -1 : 1;
			finalNormalX = axisX * sign;
			finalNormalY = axisY * sign;
			contactX = (instance.position.x + other.position.x) * 0.5;
			contactY = (instance.position.y + other.position.y) * 0.5;
		} else {
			finalNormalX = prelimNormalX;
			finalNormalY = prelimNormalY;
			if (normalLengthSignificant) {
				contactX = averageContactX;
				contactY = averageContactY;
			} else {
				contactX = maxContactX;
				contactY = maxContactY;
			}
		}
		const relativeInstanceX: number = contactX - instance.position.x;
		const relativeInstanceY: number = contactY - instance.position.y;
		const relativeOtherX: number = contactX - other.position.x;
		const relativeOtherY: number = contactY - other.position.y;
		const instanceContactVelocityX: number = instance.velocity.x - instance.angularVelocity * relativeInstanceY;
		const instanceContactVelocityY: number = instance.velocity.y + instance.angularVelocity * relativeInstanceX;
		const otherContactVelocityX: number = other.velocity.x - other.angularVelocity * relativeOtherY;
		const otherContactVelocityY: number = other.velocity.y + other.angularVelocity * relativeOtherX;
		const relativeVelocityX: number = otherContactVelocityX - instanceContactVelocityX;
		const relativeVelocityY: number = otherContactVelocityY - instanceContactVelocityY;
		const velocityAlongNormal: number = relativeVelocityX * finalNormalX + relativeVelocityY * finalNormalY;
		const shouldSkipImpulse: boolean = velocityAlongNormal > 0;
		const inverseInstanceMass: number = 1 / instance.mass;
		const inverseOtherMass: number = 1 / other.mass;
		let inverseInstInertia = 0;
		if (!instance.isStatic && instance.inertia > 1e-9) {
			inverseInstInertia = 1 / instance.inertia;
		}
		let inverseOtherInertia = 0;
		if (!other.isStatic && other.inertia > 1e-9) {
			inverseOtherInertia = 1 / other.inertia;
		}
		const relativeInstanceCrossNormal: number = relativeInstanceX * finalNormalY - relativeInstanceY * finalNormalX;
		const relativeOtherCrossNormal: number = relativeOtherX * finalNormalY - relativeOtherY * finalNormalX;
		const instanceRotationalInertia: number = relativeInstanceCrossNormal * relativeInstanceCrossNormal * inverseInstInertia;
		const otherRotationalInertia: number = relativeOtherCrossNormal * relativeOtherCrossNormal * inverseOtherInertia;
		const denominator: number = inverseInstanceMass + inverseOtherMass + instanceRotationalInertia + otherRotationalInertia;
		if (denominator > 1e-9 && !shouldSkipImpulse) {
			const impulseMagnitude: number = -2 * velocityAlongNormal / denominator;
			const impulseX: number = impulseMagnitude * finalNormalX;
			const impulseY: number = impulseMagnitude * finalNormalY;
			if (!instance.isStatic) {
				instance.velocity.x -= impulseX * inverseInstanceMass;
				instance.velocity.y -= impulseY * inverseInstanceMass;
				instance.angularVelocity -= (relativeInstanceCrossNormal * impulseMagnitude) * inverseInstInertia;
			}
			if (!other.isStatic) {
				other.velocity.x += impulseX * inverseOtherMass;
				other.velocity.y += impulseY * inverseOtherMass;
				other.angularVelocity += (relativeOtherCrossNormal * impulseMagnitude) * inverseOtherInertia;
			}
			if (!instance.isStatic && Math.abs(instance.angularVelocity) < 1e-5) {
				instance.angularVelocity = 0;
			}
			if (!other.isStatic && Math.abs(other.angularVelocity) < 1e-5) {
				other.angularVelocity = 0;
			}
		}
		let instanceMoveFactor: number = 0;
		let otherMoveFactor: number = 0;
		if (instance.isStatic) {
			otherMoveFactor = 1;
		} else if (other.isStatic) {
			instanceMoveFactor = 1;
		} else {
			const totalMass: number = instance.mass + other.mass;
			instanceMoveFactor = other.mass / totalMass;
			otherMoveFactor = instance.mass / totalMass;
		}
		const moveDistanceX: number = finalNormalX * penetrationSum;
		const moveDistanceY: number = finalNormalY * penetrationSum;
		instance.moveBy(-moveDistanceX * instanceMoveFactor, -moveDistanceY * instanceMoveFactor);
		other.moveBy(moveDistanceX * otherMoveFactor, moveDistanceY * otherMoveFactor);
	}

	public static collide(instance: Entity, other: Entity): void {
		if (instance.points === null) {
			if (other.points === null) {
				this.collideCircleCircle(instance, other);
			} else {
				this.collidePolygonCircle(other, instance);
			}
		} else {
			if (other.points === null) {
				this.collidePolygonCircle(instance, other);
			} else {
				this.collidePolygonPolygon(instance, other);
			}
		}
	}
}